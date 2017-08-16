/* DMA Routines using the RTEMS VME DMA API */

#include <stdlib.h>
#include <stdint.h>
#include <errno.h>

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
	lock=epicsEventMustCreate( epicsEventFull );

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

/*
 * Stephanie Allison found a problem (3/14/2008):
 * BLT out of a Joerger VTR10014's memory on MVME6100 resulted
 * in scrambled data.
 *
 * Here's what happened:
 *  - the VTR10014 requires 256-byte aligned addresses 
 *    for BLT (2k aligned for MBLT) (cf. their manual).
 *  - throttling mechanisms in the Tsi148 automatically
 *    and transparently break up BLTs (and MBLTs) when
 *    *either* of the conditions:
 *    a) the DMA block size is reached
 *    b) the VME bus release condition is met
 *    is satisfied.
 * Because the break-ups are not necessarily at 256b (or 2kb
 * for MBLTs) boundaries the VTR10014 sends wrong data.
 * 
 * The most stringent limitation was due to the default
 * setting of the Tsi148's VME Master control register:
 *    - default release condition: VTON timer expired OR xfer done
 *    - default VTON timer timeout: 4us. This likely limited the
 *      transfers to 64b (regardless of the DMA engine being 
 *      programmed for a larger block size).
 *
 * Workaround:
 *    - use BSP_VMEDMA_OPT_THROUGHPUT (DMA block size = 1k > 256b)
 *    - use recent RTEMS BSP (updated in 'official' CVS head on 2008/3/16)
 *      or use HACK (ONLY works on mvme6100 with Tsi148 that is mapped
 *      on VME!) [from cexp script]:
 *    
 *      vmeTsi148RegBase && (*(long*)(vmeTsi148RegBase+0x234) = 0x71b)
 *
 * IMO the VTR10014 is at fault here -- the VME standard says that
 * BLTs must not cross 256b boundaries (a rule which is observed by the
 * Tsi148) but it also says that VME slaves must latch the address on
 * the falling edge of AS -- requiring that the starting address
 * of a BLT must be 256b aligned is not explicitly allowed by the standard
 * (but not explicitly forbidden either).
 *
 * T.S, 2008/03/17
 */
uint32_t rtemsVmeDmaBusMode = BSP_VMEDMA_OPT_THROUGHPUT;

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
