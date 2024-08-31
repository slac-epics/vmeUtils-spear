#include <epicsDma.h>
#include <epicsVersion.h>
#include <stdlib.h>
#include <errno.h>

#if ((EPICS_VERSION > 3) || (EPICS_VERSION == 3 && EPICS_REVISION >= 14))

# include <epicsEvent.h>

#else

# include <vxWorks.h>
# include <semLib.h>
/*# include <memLib.h>*/
# define epicsEventId SEM_ID
# define epicsEventCreate(x) semBCreate(SEM_Q_FIFO, SEM_EMPTY)
# define epicsEventWait(x) semTake(x, WAIT_FOREVER)
# define epicsEventSignal(x) semGive(x)

#endif

/*
 * Don't cause linker errors if BSP fails to supply these routines
 */
#ifdef HAS_UNIVERSEDMA
#include <drvUniverseDma.h>
#elif defined( __rtems__) && defined(HAS_RTEMSDMASUP)
#include <drvRTEMSDmaSup.h>
#else
#ifndef vxWorks
typedef void (*VOIDFUNCPTR)(void *);
typedef unsigned long   UINT32;
#endif
typedef struct dmaRequest *DMA_ID;
#endif
typedef DMA_ID (*sysDmaCreateFunc)(VOIDFUNCPTR callback, void *context);
typedef int (*sysDmaStatusFunc)(DMA_ID dmaId);
typedef int (*sysDmaToVmeFunc)(DMA_ID dmaId, UINT32 vmeAddr, int adrsSpace,
              void *pLocal, int length, int dataWidth);
typedef int (*sysDmaFromVmeFunc)(DMA_ID dmaId, void *pLocal, UINT32 vmeAddr,
              int adrsSpace, int length, int dataWidth);
#ifdef HAS_UNIVERSEDMA
static sysDmaCreateFunc  psysDmaCreate  = universeDmaCreate;
static sysDmaStatusFunc  psysDmaStatus  = universeDmaStatus;
static sysDmaFromVmeFunc psysDmaFromVme = universeDmaFromVme;
static sysDmaToVmeFunc   psysDmaToVme   = universeDmaToVme;
#elif defined(__rtems__) && defined(HAS_RTEMSDMASUP)
static sysDmaCreateFunc  psysDmaCreate  = rtemsVmeDmaCreate;
static sysDmaStatusFunc  psysDmaStatus  = rtemsVmeDmaStatus;
static sysDmaFromVmeFunc psysDmaFromVme = rtemsVmeDmaFromVme;
static sysDmaToVmeFunc   psysDmaToVme   = rtemsVmeDmaToVme;
#else
DMA_ID sysDmaCreate(VOIDFUNCPTR callback, void *context) __attribute__((weak));
int sysDmaStatus(DMA_ID dmaId) __attribute__((weak));
int sysDmaToVme(DMA_ID dmaId, UINT32 vmeAddr, int adrsSpace,
              void *pLocal, int length, int dataWidth) __attribute__((weak));
int sysDmaFromVme(DMA_ID dmaId, void *pLocal, UINT32 vmeAddr,
              int adrsSpace, int length, int dataWidth) __attribute__((weak));
static sysDmaCreateFunc  psysDmaCreate  = sysDmaCreate;
static sysDmaStatusFunc  psysDmaStatus  = sysDmaStatus;
static sysDmaFromVmeFunc psysDmaFromVme = sysDmaFromVme;
static sysDmaToVmeFunc   psysDmaToVme   = sysDmaToVme;
#endif
/*
 * EPICS DMA identifier
 */
struct epicsDmaInfo {
    DMA_ID              dmaId;
    epicsDmaCallback_t  callback;
    void                *context;
    epicsEventId        eventId;
    int                 waiting;
};

/*
 * DMA completion callback
 */
static void
myCallback(void *context)
{
    struct epicsDmaInfo *dmaId = (struct epicsDmaInfo *)context;

    if (dmaId->waiting) {
        dmaId->waiting = 0;
        epicsEventSignal(dmaId->eventId);
    }
    if (dmaId->callback)
        (*dmaId->callback)(dmaId->context);
}

/*
 * Create a DMA handler
 */
epicsDmaId
epicsDmaCreate(epicsDmaCallback_t callback, void *context)
{
    struct epicsDmaInfo *dmaId;

    if ((psysDmaCreate == NULL)
     || (psysDmaStatus == NULL)
     || (psysDmaToVme == NULL)
     || (psysDmaFromVme == NULL))
        return NULL;
    if ((dmaId = malloc(sizeof(*dmaId))) == NULL)
        return NULL;
    if ((dmaId->dmaId = (*psysDmaCreate)(myCallback, dmaId)) == NULL) {
        free(dmaId);
        return NULL;
    }
    dmaId->eventId = NULL;
    dmaId->callback = callback;
    dmaId->context = context;
    return dmaId;
}

/*
 * Return DMA handler status
 */
int
epicsDmaStatus(epicsDmaId dmaId)
{
    return (*psysDmaStatus)(dmaId->dmaId);
}

/*
 * Start a DMA transaction to a VME module
 */
int
epicsDmaToVme(epicsDmaId dmaId, epicsUInt32 vmeAddr, int adrsSpace,
                          void *pLocal, int length, int dataWidth)
{
    return (*psysDmaToVme)(dmaId->dmaId, vmeAddr, adrsSpace, pLocal, length, dataWidth);
}

/*
 * Start a DMA transaction from a VME module
 */
int
epicsDmaFromVme(epicsDmaId dmaId, void *pLocal, epicsUInt32 vmeAddr,
                          int adrsSpace, int length, int dataWidth)
{
    return (*psysDmaFromVme)(dmaId->dmaId, pLocal, vmeAddr, adrsSpace, length, dataWidth);
}

/*
 * Start a DMA transaction to a VME module and wait for completion
 */
int
epicsDmaToVmeAndWait(epicsDmaId dmaId, epicsUInt32 vmeAddr, int adrsSpace,
                                 void *pLocal, int length, int dataWidth)
{
    int status;

    if (dmaId->eventId == NULL) {
        if ((dmaId->eventId = epicsEventCreate(epicsEventEmpty)) == NULL) {
            errno = ENOMEM;
            return -1;
        }
    }
    dmaId->waiting = 1;
    status = epicsDmaToVme(dmaId, vmeAddr, adrsSpace, pLocal, length, dataWidth);
    if (status != 0)
        return status;
    epicsEventWait(dmaId->eventId);
    return epicsDmaStatus(dmaId);
}

/*
 * Start a DMA transaction from a VME module and wait for completion
 */
int
epicsDmaFromVmeAndWait(epicsDmaId dmaId, void *pLocal, epicsUInt32 vmeAddr,
                                   int adrsSpace, int length, int dataWidth)
{
    int status;

    if (dmaId->eventId == NULL) {
        if ((dmaId->eventId = epicsEventCreate(epicsEventEmpty)) == NULL) {
            errno = ENOMEM;
            return -1;
        }
    }
    dmaId->waiting = 1;
    status = epicsDmaFromVme(dmaId, pLocal, vmeAddr, adrsSpace, length, dataWidth);
    if (status != 0)
        return status;
    epicsEventWait(dmaId->eventId);
    return epicsDmaStatus(dmaId);
}

