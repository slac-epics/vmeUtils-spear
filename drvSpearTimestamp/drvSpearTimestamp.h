/* $Id: drvSpearTimestamp.h,v 1.7 2006/10/16 23:09:37 guest Exp $ */
#ifndef DRV_SPEAR_TIME_H
#define DRV_SPEAR_TIME_H

#include <epicsTime.h>
#include <dbCommon.h>

typedef unsigned long long SpearTimestamp;
typedef int                SpearEvents;

/* a truncated timestamp: SpearTimestamp MOD 1E9 */

typedef unsigned long		SpearSmallTimestamp;

#define SPEAR_TIMESTAMP_INVALID ((SpearTimestamp)-1)
#define SPEAR_TIMESTAMP_NOW		((SpearTimestamp)-1)

#define SPEAR_TIMESTAMP_RATE	4000	/* not completely accurate nor fixed */

#define TSSM_CLOCK_KHZ			20000	/* xilinx clock for timer1/timer2    */

/* for scripts to check if this is a master or a slave;
 * set by drvSpearTimestampRegister()
 * RETURNS:
 *        1: master
 *        0: slave
 *       -1: not initialized; no TSSM present
 */
int drvSpearTimestampIsMaster();

int
drvSpearTimestampRegister(epicsUInt32 vmeAddr, int vector, int level);


/* valid MASK bits */
#define TSSM_INT_SYNC				(1<<8)
#define TSSM_INT_EVENT0				(1<<0)
#define TSSM_INT_EVENT1				(1<<1)
#define TSSM_INT_EVENT2				(1<<2)
#define TSSM_INT_EVENT3				(1<<3)
#define TSSM_INT_EVENT4				(1<<4)
#define TSSM_INT_EVENT5				(1<<5)
#define TSSM_INT_EVENT6				(1<<6)
#define TSSM_INT_EVENT7				(1<<7)

/* 
 * Connect ISR. Supply a NULL function pointer to disconnect.
 * The driver will enable the interrupt after connecting and
 * disable prior to disconnecting at the TSSM device.
 * The interrupt is also enabled at the VME level upon connection.
 * It is NOT disabled after disconnecting, however, in order
 * not to interfere with other devices sharing the same level.
 *
 * The 'mask' argument selects interupt sources to activate.
 * A bitmask of raised interrupts is passed to the ISR.
 */
int
drvSpearTimestampConnectISR(void (*isr)(void*arg, unsigned mask),void *arg, unsigned mask);

/* read the current timestamp;
 * returns -1 if no module is installed
 *         -2 if it is out of sync and
 *          0 on success.
 * *pres is set to SPEAR_TIMESTAMP_INVALID on error.
 */
int
spearTimestampGetCurrent(SpearTimestamp *pres);

/* convenience routine to obtain an easier-to handle
 * short timestamp (no long long at the CEXP prompt)
 */
SpearSmallTimestamp
spearSmallTimestampGetCurrent();

/* Set the processing time of a record
 * (NOTE: TSE must be set to epicsTimeEventDeviceTime (-2) )
 * - to be used by device support modules.
 * prec->time.sec is set to the current wallclock time
 * prec->time.nsec to the spear timestamp (modulo 1E9).
 * 
 * RETURNS: current timestamp or SPEAR_TIMESTAMP_INVALID
 */
SpearTimestamp
spearTimestampSetRecordTimeCurrent(dbCommon *prec);

/* if tse == -2, set the processing time to 'now' (secs) / 'tstamp' (nsecs).
 * If the 'doAdjSecs' flag is set, the seconds are adjusted by the difference
 * of the current timestamp and 'tstamp'.
 */ 
SpearTimestamp
spearTimestampSetRecordTime(dbCommon *prec, SpearTimestamp tstamp, int doAdjSecs);

/* number of events (0..6) */
#define	SPEAR_TIMESTAMP_EVENT_NUM	7
/* Return the real-time event set or a negative number
 * if no TSSM is present. A negative result is guaranteed to
 * have no event bit set.
 */
SpearEvents
spearTimestampGetEvents();

/* set event bit 'num' to 'val' and return the new event flag word
 * (result contains also bits other than 1<<num)
 * RETURNS -1 on error (no TSSM, not a master or illegal event number)
 */
SpearEvents
spearTimestampSetEvent(int num, int val);

/* return spearTime modulo nsec/sec
 * TODO: what to do when spearTime rolls over??
 */
#define spearTimestampToEpicsNsec(s) \
	(SPEAR_TIMESTAMP_INVALID == s ? 1000000000 : ((epicsUInt32)(s % (SpearTimestamp)1000000000)))

/* Return time (in microseconds) that expired since the last TSSM 'tick'/interrupt */
unsigned long
spearTimestampGetUsecSinceTick();

/* for profiling/performance: read the TSSM timer1
 * with reference to the PPC timebase. I.e., 
 * you may fix the reference by
 *   ref = spearTimestampTBbackdate();
 *   / * 'ref' equals the TB value as of the last spear timestamp * /
 *   xxxxx
 *   yyyyy
 *   MFTB(val); / * read TB * /
 *   val - ref; / * time expired since the last timestamp in TB ticks * /
 */
unsigned long
spearTimestampTBbackdate();

#endif
