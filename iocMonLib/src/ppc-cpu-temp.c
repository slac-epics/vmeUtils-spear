/* $Id: ppc-cpu-temp.c,v 1.2 2006/08/25 17:19:23 saa Exp $ */

/* read the PowerPC core temperature using the TAU (thermal assist unit) */

/* Author: Till Straumann <strauman@slac.stanford.edu>, 2003 */

/* Useful information can be found in motorola's AN1800
 * 
 * This driver implements a 'read-method' for the 'devBusMapped'
 * device support layer. The temperature value is read into a record
 * using asynchronous two phase processing:
 * Phase 1 triggers a number of successive approximation cycles which
 * are implemented using an iteration of delayed callback invocations 
 * (executed in the context of the low priority EPICS callback task).
 * Once the SAR sequence completes, the temperature is stored to the
 * record and Phase 2 processing is scheduled.
 */

#ifdef __PPC__

#include <rtems.h>
#include <libcpu/spr.h>

#include <stdlib.h>
#include <stdio.h>

#include <alarm.h>
#include <dbDefs.h>
#include <dbAccess.h>
#include <recGbl.h>
#include <callback.h>

#include <dbCommon.h>

#include <registry.h>

#include <devBusMapped.h>

#define DEBUG

SPR_RW(THRM1)
SPR_RW(THRM2)
SPR_RW(THRM3)

#ifdef DEBUG
unsigned read_THRM1()
{ return _read_THRM1(); }
unsigned read_THRM2()
{ return _read_THRM2(); }
unsigned read_THRM3()
{ return _read_THRM3(); }

void write_THRM1(unsigned val)
{ _write_THRM1(val); }
void write_THRM2(unsigned val)
{ _write_THRM2(val); }
void write_THRM3(unsigned val)
{ _write_THRM3(val); }
#endif

/* resolution reduction by a number of bits
 * I believe the sensor has only 4degC resolution hence
 * a value of 2 should be ok...
 */
#define THRM_THRESH_BITS_RES 0

#define THRM_THRESH_BITS (7 - THRM_THRESH_BITS_RES)
#define THRM_THRESH_SHIFT (31-(1+THRM_THRESH_BITS))
#define THRM_THRESH_MASK ((1<<THRM_THRESH_BITS)-1)

#define THRMX_V			(1<<(31-31))
#define THRMX_TIE		(1<<(31-30))
#define THRMX_TID		(1<<(31-29))
#define THRMX_THRESH(v)	(((v)&THRM_THRESH_MASK)<<THRM_THRESH_SHIFT)
#define THRMX_TIV		(1<<(31-1))
#define THRMX_TIN		(1<<(31-0))

#if defined(THRM3_E) /* spr.h value perhaps still buggy */
#warning has libcpu/score/cpu/powerpc/rtems/powerpc/registers.h been fixed??
#undef THRM3_E
#endif

#ifdef THRM3_SITV
/* we don't want the predefined version which is just a mask */
#undef THRM3_SITV
#endif
#define THRM3_E			(1<<(31-31))
#define THRM3_SITV(v)	(((v) & 0x1fff)<<(31-30))

static CALLBACK cbk;

static int tempOffset = 0;

static void
setupTHRM3(int forceThrm3Init)
{
	if ( ! (THRM3_E & _read_THRM3()) || forceThrm3Init ) {
		if (forceThrm3Init)
			_write_THRM3(forceThrm3Init);
		else
			/* just use maximum delay */
			_write_THRM3(THRM3_SITV(0x1fff) | THRM3_E);
			/* NOTE: the calibration bits 0:6 are
			 *       a) undocumented
			 *       b) don't seem to do what AN1800 suggests,
			 *          i.e. I don't observe a weight of 20/8/4/2 degC
			 *          but quite larger values :-(
			 */
	} /* else assume someone has set it up correctly */
}


/* return current DAC value corrected by the comparator result;
 * i.e. if the analog value was less than the DAC, remove
 * the test bit from the result
 */
static inline int
readAdjust(int testbit)
{
int	status = _read_THRM1();
int rval;

	if ( ! (status & THRMX_TIV) )
		return -1;

	rval = (status >> THRM_THRESH_SHIFT) & THRM_THRESH_MASK;

	/* temp was less than threshold; remove test bit */
	if (! (status & THRMX_TIN))
		rval &= ~testbit;
	return rval;	
}

/* trigger the DAC and schedule a delayed callback for reading
 * the comparator result
 */
static void
startDAC(unsigned val)
{
	_write_THRM1( THRMX_THRESH(val) | THRMX_V );
	callbackRequestDelayed(&cbk, 0.001);
}


/* Evaluate the previous successive approximation step and
 * start the next one if necessary by chaining delayed
 * callbacks to this routine.
 */
static void
thermCB(struct callbackPvt *arg)
{
int				tmp;
DevBusMappedPvt pvt;
int				bit;

	callbackGetUser(pvt, arg);

	bit = (int)pvt->udata;

	/* read the DAC value and remove 'bit' if the analog
	 * value was less than the threshold.
	 */
	if ( (tmp = readAdjust(bit)) < 0 ) {
		bit = 0;	/* error; abort */
	} else {
		bit >>= 1;	/* try next bit */
	}

	/* more approximation steps needed ? */
	if (bit) {
		/* yes */
		pvt->udata = (void*)bit;
		/* startDAC will schedule this routine to be called again */
		startDAC(tmp | bit);
	} else {
		/* callback chain terminates; arrange for the record being
		 * processed again (async processing phase 2)
		 */
		pvt->udata = (void*)(tmp << THRM_THRESH_BITS_RES);
		callbackRequestProcessCallback(&cbk, priorityLow, pvt->prec);
	}
}

static int
readTemp(DevBusMappedPvt pvt, unsigned *pval , dbCommon *prec)
{
	if ( !prec->pact ) {
		prec->pact = 1;
		/* init phase */
		callbackSetCallback(thermCB,     &cbk);
		callbackSetPriority(priorityLow, &cbk);
		callbackSetUser(pvt,             &cbk);
		/* store the successive approximation test bit in 'udata'
		 * - when the approximation terminates, the callback will
		 * store the result in 'udata'
		 */
		pvt->udata = (void*)(1 << (THRM_THRESH_BITS-1));
		/* startDAC schedules chain of callbacks executing
		 * successive approximation steps. Eventually, a final
		 * callback will cause this record to be processed again...
		 */
		startDAC((int)pvt->udata);
	} else {
		/* completion phase */
		if ((int)pvt->udata < 0) {
			/* read failed */
			recGblSetSevr(prec, READ_ALARM, INVALID_ALARM);
		} else {
			*pval = (unsigned)pvt->udata;
				*pval += tempOffset;
		}
		prec->pact = 0;
	}
	return 0;
}

/* struct with our access 'methods' - we provide just a read routine */
static DevBusMappedAccessRec tempAccess = {
	readTemp,
	0
};

int
ppcCpuTempInit(int forceThrm3Init)
{
char *off_str;
int  off;

	/* Try to retrieve an offset calibration from the environment */
	if ( (off_str = getenv("PPC_CPU_TAU_OFFSET")) &&
				 ( 1 == sscanf(off_str,"%i",&off) ) )
		tempOffset = off;
	/* setup CPU THERM3 special register */
	setupTHRM3(forceThrm3Init);

	/* all we need is our special 'read' routine */
	devBusMappedRegisterIO("tau", &tempAccess);
	return 0;
}
#endif
