/*
	Modified:	Shantha Condamoor
	Date:		1-Jun-2011
	Author:		Till Straumann
	Patch:		BUGFIX: MUST NOT use a mutex for a lock since it is not released from
   				the task context which acquired it!
   				This bug caused a task to never relinquish a temporarily inherited,
   				high priority (since it apparently always held this driver's mutex).
	
*/
/* DMA Routines using the RTEMS VME DMA API */

#include <stdlib.h>
#include <stdint.h>
#include <errno.h>
#include <assert.h>

#include <epicsEvent.h>
#include <epicsInterrupt.h>
#include <errlog.h>
#include <devLib.h>

#include <bsp.h>
#include <bsp/VMEDMA.h>

#include <drvRTEMSDmaSup.h>

#ifndef PCI_DRAM_OFFSET
#define PCI_DRAM_OFFSET 0
#endif
#define LOCAL2PCI(adrs) ((unsigned long)(adrs)+(PCI_DRAM_OFFSET))

#undef DEBUG

#define DMACHANNEL       0

static epicsEventId lock=0;
static DMA_ID inProgress=0;

typedef struct dmaRequest {
		VOIDFUNCPTR				callback;
		void					*closure;
		uint32_t				status;
		uint32_t				mode;
} DmaRequest;

#ifdef DEBUG
unsigned long vmeDmaLastStatus=0;
#endif

static void
rtemsVmeDmaIsr(void *p)
{
unsigned long s=BSP_VMEDmaStatus(DMACHANNEL);

#ifdef DEBUG
	vmeDmaLastStatus=s;
#endif

	if (inProgress) {
		inProgress->status = s;
		if (inProgress->callback)
				inProgress->callback(inProgress->closure);
		inProgress = 0;
	}
	/* yield the driver */
	epicsEventSignal(lock);
}

static void
rtemsVmeDmaInit(void)
{
	lock=epicsEventMustCreate(epicsEventFull);

	/* connect and enable DMA interrupt */
	assert( 0==BSP_VMEDmaInstallISR(DMACHANNEL,rtemsVmeDmaIsr,0) );

}

DMA_ID
rtemsVmeDmaCreate(VOIDFUNCPTR callback, void *context)
{
DMA_ID	rval;

	/* lazy init */
	if (!lock) {
		rtemsVmeDmaInit();
	}

	rval = malloc(sizeof(*rval));
	rval->callback = callback;
	rval->closure = context;
	rval->status  = -1;
	rval->mode    = 0;

	return rval;
}

STATUS
rtemsVmeDmaStatus(DMA_ID dmaId)
{
	return dmaId->status ? EIO : 0;
}

uint32_t
rtemsVmeDmaStatusRaw(DMA_ID dmaId)
{
	return dmaId->status;
}

static __inline__ uint32_t
dw2mode(int w)
{
	switch (w) {
		case 1:	return VME_MODE_DBW8;
		case 2:	return VME_MODE_DBW16;
		case 4: return VME_MODE_DBW32;
		default:
			break;
	}
	return 0;
}

uint32_t rtemsVmeDmaBusMode = BSP_VMEDMA_OPT_SHAREDBUS;

static STATUS
rtemsVmeDmaStart(DMA_ID dmaId, uint32_t mode, void *pLocal, UINT32 vmeAddr, int length)
{
STATUS rval;

	dmaId->status = -1;

	epicsEventWait( lock );

	if ( mode != dmaId->mode ) {
		rval = BSP_VMEDmaSetup( DMACHANNEL, rtemsVmeDmaBusMode, mode, 0 );
		if ( rval ) {
			epicsEventSignal( lock );
			return rval;
		}
		dmaId->mode = mode;
	}

	inProgress = dmaId;

	rval = BSP_VMEDmaStart( DMACHANNEL, LOCAL2PCI(pLocal), vmeAddr, length );

	if ( rval ) {
		inProgress = 0;
		epicsEventSignal( lock );
	}
	
	return rval;
}

STATUS
rtemsVmeDmaFromVme(DMA_ID dmaId, void *pLocal, UINT32 vmeAddr,
	int adrsSpace, int length, int dataWidth)
{
uint32_t mode = adrsSpace | dw2mode( dataWidth );

	return rtemsVmeDmaStart(dmaId, mode, pLocal, vmeAddr, length);

}

STATUS
rtemsVmeDmaToVme(DMA_ID dmaId, UINT32 vmeAddr, int adrsSpace,
				void *pLocal, int length, int dataWidth)
{
uint32_t mode = adrsSpace | dw2mode( dataWidth ) | BSP_VMEDMA_MODE_PCI2VME;

	return rtemsVmeDmaStart(dmaId, mode, pLocal, vmeAddr, length);
}
