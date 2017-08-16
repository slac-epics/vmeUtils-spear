/* $Id: drvRTEMSDmaSup.h,v 1.2 2008/03/15 00:13:58 saa Exp $ */

/* EPICS glue to the RTEMS VMEDMA API */

#ifndef DRV_RTEMS_VME_DMA_SUP_H
#define DRV_RTEMS_VME_DMA_SUP_H

#include <stdint.h>

typedef void 		(*VOIDFUNCPTR)();
typedef uint32_t UINT32;
typedef uint16_t UINT16;
typedef int      STATUS;

typedef struct dmaRequest *DMA_ID;

DMA_ID
rtemsVmeDmaCreate(VOIDFUNCPTR callback, void *context);

STATUS
rtemsVmeDmaStatus(DMA_ID dmaId);

/* retrieve the raw status (as passed from device) of a terminated DMA */
uint32_t
rtemsVmeDmaStatusRaw(DMA_ID dmaId);

STATUS
rtemsVmeDmaFromVme(DMA_ID dmaId, void *pLocal, UINT32 vmeAddr,
	int adrsSpace, int length, int dataWidth);

STATUS
rtemsVmeDmaToVme(DMA_ID dmaId, UINT32 vmeAddr, int adrsSpace,
    void *pLocal, int length, int dataWidth);

#endif
