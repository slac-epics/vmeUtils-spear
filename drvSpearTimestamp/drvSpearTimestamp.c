/* $Id: drvSpearTimestamp.c,v 1.14 2011/01/24 18:50:40 strauman Exp $ */

#include "drvSpearTimestamp.h"
#include "basicIoOps.h"
#include "drvSup.h"
#include "devLib.h"
#include "errlog.h"
#include "devBusMapped.h"
#include "iocsh.h"
#include "epicsExport.h"

#define myprintf errlogPrintf

typedef volatile epicsUInt32 TSSMRegister;

typedef struct TSSMRegsRec_  {
	TSSMRegister csr;
	TSSMRegister divisor;
	TSSMRegister tsHi;
	TSSMRegister tsLo;
	TSSMRegister dely2;
	TSSMRegister dely1;
	TSSMRegister outputs;
	TSSMRegister eventFlags;
	TSSMRegister timer1;
	TSSMRegister timer2;
	TSSMRegister softSync;
	TSSMRegister inputs;
	TSSMRegister intEnable;
	TSSMRegister intStatus;
	TSSMRegister dig2Ctrl;
} TSSMRegsRec, *TSSMRegs;

static TSSMRegs		   tssm = 0;
static int             isMaster = -1;
static epicsUInt32	   tssmVME;
int             	   tssmIRQVector = 0, tssmIRQLevel = 0;
static DevBusMappedDev devBM = 0;

#define TSSM_CSR_RS232_TS			(1<<25)
#define TSSM_CSR_CLK_SEL_INT		(1<<24)
#define TSSM_CSR_TS_VALID			(1<<23)
#define TSSM_CSR_TS_SYNC_ERR		(1<<22)
#define TSSM_CSR_TS_PARITY_ERR		(1<<21)
#define TSSM_CSR_RS232_PARITY_ERR	(1<<20)
#define TSSM_CSR_IRQ_FLAG			(1<<19)
#define TSSM_CSR_IRQ_OVERRUN		(1<<18)
#define TSSM_CSR_FRONT_IO			(1<<17)
#define TSSM_CSR_MASTER 			(1<<16)
#define TSSM_CSR_IRQ_VEC(vec) 		(((vec)&0xff)<<8)
#define TSSM_CSR_IRQ_LVL(lvl) 		(((lvl)&0x07)<<5)
#define TSSM_CSR_IRQ_ENA	 		(1<<4)
#define TSSM_CSR_SYNC_SRC_MSK		(3<<2)
#define TSSM_CSR_SYNC_SRC_INT		(3<<2)
#define TSSM_CSR_SYNC_SRC_SOFT		(2<<2)
#define TSSM_CSR_SYNC_SRC_SYNC2		(1<<2)
#define TSSM_CSR_SYNC_SRC_TS		(0<<2)
#define TSSM_CSR_SYNC2_OUT_ENA		(1<<1)
#define TSSM_CSR_SYNC1_OUT_ENA		(1<<0)

#define TSSM_INT_MASK				(0x17f) /* SYNC and events 0..7 */

#define EVENT_MASK ((1<<SPEAR_TIMESTAMP_EVENT_NUM)-1)

#define TSSM_EVENTS_GET(eventFlags)		(((eventFlags)>>(isMaster ? 8 : 16))&EVENT_MASK)
#define TSSM_EVENTS_SET(eventFlags)		(((eventFlags)&EVENT_MASK)<<8)

static int
tssm_rd(DevBusMappedPvt pvt, unsigned *pvalue, dbCommon *prec)
{
	*pvalue = in_be32( pvt->addr );
	return 0;
}

static int
tssm_wr(DevBusMappedPvt pvt, unsigned value, dbCommon *prec)
{
	out_be32( pvt->addr, value );
	spearTimestampSetRecordTimeCurrent( prec );
	return 0;
}

static DevBusMappedAccessRec tssmIO = {
	tssm_rd,
	tssm_wr,
};


int
spearTimestampGetCurrent(SpearTimestamp *pres)
{
epicsUInt32 hi,lo;
	if ( !tssm ) {
		*pres = SPEAR_TIMESTAMP_INVALID;
		return -1;
	}
	if ( (in_be32(&tssm->csr) & TSSM_CSR_TS_VALID) ) {
		hi = in_be32(&tssm->tsHi);
	    lo = in_be32(&tssm->tsLo);
		*pres = (((SpearTimestamp)hi)<<32) | ((SpearTimestamp)lo);
		return 0;
	}
	*pres = SPEAR_TIMESTAMP_INVALID;
	return -2;
}

SpearSmallTimestamp
spearSmallTimestampGetCurrent()
{
SpearTimestamp ts;
	return spearTimestampGetCurrent(&ts) ? (SpearSmallTimestamp)-1 : spearTimestampToEpicsNsec(ts);
}

/* DONT ACCESS VME if doAdjSecs == 0 */
SpearTimestamp
spearTimestampSetRecordTime(dbCommon *pr, SpearTimestamp tstamp, int doAdjSecs)
{
SpearTimestamp rval = SPEAR_TIMESTAMP_INVALID;
	if ( epicsTimeEventDeviceTime == pr->tse ) {
		epicsTimeGetEvent(&pr->time, epicsTimeEventCurrentTime);
		pr->time.nsec = spearTimestampToEpicsNsec(tstamp);
		if ( doAdjSecs ) {
			/* should we preserve the original nsecs in case this fails? */
			spearTimestampGetCurrent( &rval );
			pr->time.secPastEpoch -= (rval - tstamp)/SPEAR_TIMESTAMP_RATE;
		}
	}
	return rval;
}

SpearTimestamp
spearTimestampSetRecordTimeCurrent(dbCommon *pr)
{
SpearTimestamp rval = SPEAR_TIMESTAMP_INVALID;
	if ( epicsTimeEventDeviceTime == pr->tse ) {
		epicsTimeGetEvent(&pr->time, epicsTimeEventCurrentTime);
		/* should we preserve the original nsecs in case this fails? */
		spearTimestampGetCurrent( &rval );
		pr->time.nsec = spearTimestampToEpicsNsec(rval);
	}
	return rval;
}

int drvSpearTimestampReport(int level)
{
	if ( tssm ) {
		epicsUInt32 csr = in_be32( &tssm->csr );
		myprintf("TSSM Driver; module found @%p (VME address A24: 0x%08x)\n",
				tssm, tssmVME);
		if ( level > 0 ) {
			myprintf("  Card is in");
			if ( csr & TSSM_CSR_MASTER ) {
				myprintf("MASTER");
			} else {
				myprintf("Receiver");
			}
			myprintf(" mode; ");
			if ( csr & TSSM_CSR_FRONT_IO ) {
				myprintf("Front Panel");
			} else {
				myprintf("Rear");
			}
			myprintf(" IO configured\n");
		}
		if ( csr & TSSM_CSR_TS_SYNC_ERR ) {
			myprintf("  WARNING: Sync error detected\n");
		}
		if ( csr & TSSM_CSR_TS_PARITY_ERR ) {
			myprintf("  WARNING: TS Parity error detected\n");
		}
	} else {
		myprintf("No TSSM card registered\n");
	}
	return 0;
}

int
drvSpearTimestampIsMaster()
{
	return isMaster;
}

int
drvSpearTimestampRegister(epicsUInt32 vmeAddr, int vector, int level)
{
epicsUInt32 csr;
int master;
	if ( devRegisterAddress(
				"drvSpearTime",
				atVMEA24,
				vmeAddr,
				256,
				(void*)&tssm) ) {
		tssm = 0;
		errlogPrintf("drvSpearTime: Unable to register A24 address 0x%08x\n",vmeAddr);
		return -1;
	}
	if ( devReadProbe( sizeof(csr), &tssm->csr, &csr ) ) {
		errlogPrintf("drvSpearTime: Read-probe of %p (@VME24 0x%08x) failed\n", 
					&tssm->csr, vmeAddr );
		tssm = 0;
		goto cleanup;
	}
	csr = in_be32(&tssm->csr);

	master =  ( csr & TSSM_CSR_MASTER ) ? 1 : 0;

#if 0 /* disable paranoia for now -- allow slave to come up before the master sends timestamps */
	if ( ! (csr & (TSSM_CSR_TS_VALID | TSSM_CSR_TS_SYNC_ERR | TSSM_CSR_TS_PARITY_ERR)) && ! master ) {
		errlogPrintf("drvSpearTime: CSR value 0x%08x makes no sense - not a TSSM?\n", csr);
		tssm = 0;
		goto cleanup;
	}
#endif

	errlogPrintf("drvSpearTime: registered at %p (@VME24 0x%08x)\n", tssm, vmeAddr);
	tssmVME = vmeAddr;

	if ( level ) {
		csr &= ~(TSSM_CSR_IRQ_VEC(0xff) | TSSM_CSR_IRQ_LVL(0x7) | TSSM_CSR_IRQ_ENA);
		csr |= TSSM_CSR_IRQ_VEC(vector) | TSSM_CSR_IRQ_LVL(level);
	}

	/* if it's the master initialize vital bits already here, just
	 * to make sure we have the global 4kHz master running even if
	 * EPICS initialization fails (driver init might not execute)...
	 */
	if ( master ) {
		errlogPrintf("drvSpearTimestampRegister: This TSSM card is jumpered to be a master\n");
		csr &= ~(TSSM_CSR_CLK_SEL_INT | TSSM_CSR_SYNC_SRC_MSK);
		csr |= ( TSSM_CSR_SYNC_SRC_INT | TSSM_CSR_SYNC1_OUT_ENA | TSSM_CSR_SYNC2_OUT_ENA);
		out_be32( &tssm->divisor, 319 );
	} 

	if ( master || level ) {
		/* don't touch CSR on a slave if no level is configured; this allows
		 * secondary CPUs to share a TSSM
		 */
		out_be32( &tssm->csr, csr );
	}

	devBM = devBusMappedRegister( master ? "tssmMas" : "tssmSlv", (volatile void*)tssm );

	if ( master ) {
		devBusMappedRegisterIO( "tssmIO", &tssmIO );
	}

	isMaster = master;

cleanup:
	if (!tssm) {
		devUnregisterAddress(
				atVMEA24,
				vmeAddr,
				"drvSpearTime");
		return -1;
	}
	tssmIRQVector = vector;
	tssmIRQLevel  = level;

	return 0;
}

static void (*theisr)() = 0;
static unsigned themask = 0;

static void tssmWrap(void *arg)
{
unsigned mask = in_be32( &tssm->intStatus );
	
	if ( theisr )
		theisr(arg, mask & TSSM_INT_MASK);

	out_be32( &tssm->intStatus, mask );
}

int
drvSpearTimestampConnectISR(void (*isr)(void*, unsigned),void *arg, unsigned mask)
{
int rval;
	if ( !tssm ) {
		errlogPrintf("drvSpearTimestamp: no TSSM registered\n");
		return -1;
	}
	if ( !mask && isr ) {
		errlogPrintf("must supply interrupt sources\n");
		return -1;
	} 
	if ( !tssmIRQVector || !tssmIRQLevel ) {
		errlogPrintf("drvSpearTimestamp: registered without valid VME interrupt vector (was %i) or level (was %i)\n",
				tssmIRQVector, tssmIRQLevel);
		return -1;
	}
	if ( isr ) {
		if ( (rval = devConnectInterruptVME(tssmIRQVector, tssmWrap, arg) ) )
			return rval;
		themask = mask & TSSM_INT_MASK;
		theisr  = isr;
		/* clear pending irqs */
		out_be32( &tssm->intStatus, 0xffffffff );
		/* enable at TSSM */
		out_be32( &tssm->csr, in_be32( &tssm->csr ) | TSSM_CSR_IRQ_ENA );
		out_be32( &tssm->intEnable, themask );
		devEnableInterruptLevelVME(tssmIRQLevel);
	} else {
		/* leave VME level on in case other devices use it! */
		themask = 0;
		theisr  = 0;
		out_be32( &tssm->csr, in_be32( &tssm->csr ) & ~TSSM_CSR_IRQ_ENA );
		out_be32( &tssm->intEnable, 0 );
		out_be32( &tssm->intStatus, TSSM_INT_MASK );
		rval = devDisconnectInterruptVME(tssmIRQVector, tssmWrap);
	}
	return rval;
}

SpearEvents
spearTimestampGetEvents()
{
	return tssm ? TSSM_EVENTS_GET(in_be32(&tssm->eventFlags)) : -(1<<SPEAR_TIMESTAMP_EVENT_NUM);
}


int
spearTimestampSetEvent(int num, int val)
{
unsigned flags;
	if ( 1 != isMaster || num < 0 || num >= SPEAR_TIMESTAMP_EVENT_NUM )
		return -1;
	num = 1<<num;
	epicsMutexLock( devBM->mutex );
	flags = TSSM_EVENTS_GET(in_be32(&tssm->eventFlags));
	if ( val )
		flags |= num;
	else
		flags &= ~num;
	out_be32(&tssm->eventFlags, TSSM_EVENTS_SET(flags));
	epicsMutexUnlock( devBM->mutex );
	return flags;
}

#if defined(__rtems__) && defined(__PPC__)
static unsigned tbkHz;
#endif

unsigned long
spearTimestampTBbackdate()
{
#if defined(__rtems__) && defined(__PPC__)
unsigned flags, rval, tssmTime;
	rtems_interrupt_disable(flags);
		asm volatile("mftb %0":"=r"(rval));	
		tssmTime = in_be32( &tssm->timer1 );
	rtems_interrupt_enable(flags);
	tssmTime *= tbkHz;
	return rval - tssmTime/TSSM_CLOCK_KHZ;
#else
#warning TBbackdate not implemented on your CPU and/or OS
	return 0xdeadbeef;
#endif
}

unsigned long
spearTimestampGetUsecSinceTick()
{
	return in_be32( &tssm->timer1 ) / (TSSM_CLOCK_KHZ/1000);
}

static int
drvSpearTimestampInit()
{
epicsUInt32 csr;
	if (!tssm) {
		errlogPrintf("No TSSM card registered\n");
		return -1;
	}

#if defined(__rtems__) && defined(__PPC__)
	tbkHz = BSP_bus_frequency / BSP_time_base_divisor;
#endif

	csr = in_be32( &tssm->csr );
	csr &= ~(TSSM_CSR_CLK_SEL_INT | TSSM_CSR_SYNC_SRC_MSK);

	if ( isMaster ) {
		errlogPrintf("drvSpearTimestamp: This TSSM card is jumpered to be a master\n");
		csr |= TSSM_CSR_SYNC_SRC_INT;
		out_be32( &tssm->divisor, 319 );
	} else {
		csr |= TSSM_CSR_SYNC_SRC_TS;
	}

	csr |= ( TSSM_CSR_SYNC1_OUT_ENA | TSSM_CSR_SYNC2_OUT_ENA);

	/* don't touch CSR if no irq level is configured -- this means
	 * we are listening to a card that is controlled by another CPU
	 */
	if ( tssmIRQLevel )
		out_be32( &tssm->csr, csr );
	return 0;
}

struct drvet drvSpearTimestamp = {
	2,
	(DRVSUPFUN) drvSpearTimestampReport,
	(DRVSUPFUN) drvSpearTimestampInit,
};
epicsExportAddress(drvet, drvSpearTimestamp);

/*******************************************************************************
* EPICS iocsh Command registry
*/


/* drvSpearTimestampReport(int level) */
static const iocshArg drvSpearTimestampReportArg0 = {"level", iocshArgInt};
static const iocshArg * const drvSpearTimestampReportArgs[1] = {&drvSpearTimestampReportArg0};
static const iocshFuncDef drvSpearTimestampReportFuncDef =
    {"drvSpearTimestampReport",1,drvSpearTimestampReportArgs};
static void drvSpearTimestampReportCallFunc(const iocshArgBuf *args)
{
    drvSpearTimestampReport(args[0].ival);
}

/* drvSpearTimestampRegister(epicsUInt32 vmeAddr, int vector, int level) */
static const iocshArg drvSpearTimestampRegisterArg0 = {"vmeAddr", iocshArgInt};
static const iocshArg drvSpearTimestampRegisterArg1 = {"vector", iocshArgInt};
static const iocshArg drvSpearTimestampRegisterArg2 = {"level", iocshArgInt};
static const iocshArg * const drvSpearTimestampRegisterArgs[3] = {
    &drvSpearTimestampRegisterArg0, &drvSpearTimestampRegisterArg1,
    &drvSpearTimestampRegisterArg2};
static const iocshFuncDef drvSpearTimestampRegisterFuncDef =
    {"drvSpearTimestampRegister",3,drvSpearTimestampRegisterArgs};
static void drvSpearTimestampRegisterCallFunc(const iocshArgBuf *arg)
{
    drvSpearTimestampRegister(arg[0].ival, arg[1].ival, arg[2].ival);
}

LOCAL void drvSpearTimestampRegistrar(void) {
    iocshRegister(&drvSpearTimestampReportFuncDef,drvSpearTimestampReportCallFunc);
    iocshRegister(&drvSpearTimestampRegisterFuncDef,drvSpearTimestampRegisterCallFunc);
}
epicsExportRegistrar(drvSpearTimestampRegistrar);
