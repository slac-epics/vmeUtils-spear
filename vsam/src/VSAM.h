/* VSAM.h - Definitions common to device and driver code
 *		for VME Smart Analog Monitor.
 *
 *	Author:		Susanna Jacobson
 *	Date:		10-30-97
 *
 * Mods: 
 *        05-Aug-05, K. Luchini
 *            Add definition for OK,ERROR and changed VOID to void
 *        09-Mar-01, K. Luchini 
 *            Changed VSAM_HARDWARE_REV from 1 to 3
 *
 */
#ifndef VSAM_H
#define VSAM_H
#ifdef __cplusplus
extern "C" {
#endif  /* __cplusplus */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#include <epicsVersion.h>
#if EPICS_VERSION>=3 && EPICS_REVISION>=14

#include <epicsMutex.h>
#include <epicsThread.h>
#include <epicsString.h>
#include <epicsInterrupt.h>
#include <cantProceed.h>
#include <epicsExport.h>
#include <drvSup.h>
#include <dbScan.h>
#include <ellLib.h>

#else
#error "You need EPICS 3.14 or above because we need OSI support!"
#endif

#define VSAM_DRV_VERSION "VSAM driver V1.0"

#define VSAM_MAX_CARDS    16
#define VSAM_NUM_CHANS    32
#define VSAM_HARDWARE_REV 3           /* changed form 1 to 3 */
#define VSAM_BASE_ADDRS   0x400000

#ifndef OK
#define OK 0
#endif

#ifndef ERROR
#define ERROR -1
#endif

/* "channel" numbers for the four status and control registers */
#define RESET_CHANNEL   32              /* VSAM reset register           */
#define MODE_CHANNEL    33              /* Mode Control Register         */
#define STATUS_CHANNEL  34              /* Status Register               */
#define DIAG_CHANNEL    35              /* Diagnostic Test Mode register */

/* data types for ai_read */
#define DATA_TYPE       'D'             /* analog data (raw val is float)   */
#define RANGE_TYPE      'R'             /* channel range (raw val is long)  */
#define AC_TYPE         'A'             /* AC measurement (raw val is long) */
#define CSR_TYPE        'B'             /* binary status or control register */

/* bits in Mode Control Register */
#define SET_FAST_SCAN   0x00000001      /* 0: normal scan; 1: fast scan       */
#define SET_FIRMWARE    0x00000002      /* 0: analog ch data; 1: firmware rev */
#define SET_LITTLE_END  0x00000004      /* 0: big-endian; 1: little-endian    */
#define MODE_MASK       0x00000007      /* OR of bits used                    */

/* bits in Status Register */
#define FAST_SCAN_MODE  0x00000001      /* 0: normal scan; 1: fast scan       */
#define FIRMWARE_REV    0x00000002      /* 0: analog ch data; 1: firmware rev */
#define LITTLE_END_MODE 0x00000004      /* 0: big-endian; 1: little-endian    */
#define CALIB_SUCCESS   0x00000008      /* 1: internal calibration succeeded  */
#define LEND_STS_MASK   0x0f000000      /* sts reg mask in little-endian mode */

/* voltage ranges corresponding to range byte values */
#define MAX_RANGE_BYTE  10

/* divisor used for AC measurements.                   */
/* AC peak-to-peak voltage is ranges[range]*ac/16384   */
#define AC_DIVISOR      16384.0               /* 2**14 */

typedef struct VSAMPVT {
	short		lchan;
	unsigned long	mask;
	int		shift;
	void		*prange;
} VSAMPVT;


/* memory structure of the VSAM - 256 bytes total */
#define VSAM_MEM_SIZE    256
#define VSAM_STATUS_REG  127            /* offset from base */

typedef volatile struct  {
	float		data[32];	/* four-byte floating point values */
	unsigned char	range[32];	/* channel range - AC calc's and diag */
	unsigned short	ac[32];		/* AC measurement readout */
	unsigned long	reset;		/* write to reset VSAM */
	unsigned long	mode_control;	/* sets scan and data modes */
	unsigned long	status;		/* status register */
	unsigned long	pad;
	unsigned long	diag_mode;	/* write for diagnostic test mode */
	unsigned long	padding[3];
} VSAMMEM;

typedef ELLLIST VSAM_CARD_LIST;

typedef struct VSAMCNFG {
  ELLNODE         node;  
  VSAMMEM        *pVSAM;
  unsigned short  card;
  unsigned short  present;
  unsigned short  registered;
  unsigned long   bus_addr;      /* system bus address */
  /* 
   * To obtain the firmware version number you first write
   * request to read this information by setting bit D1 (0-31) of
   * the MODE CONTROL REGISTER. After writing to this register the
   * firmware version number returned in all channels in floating point format.
   * Since the firmware version is the same for all channels on the board
   * only channel 0 is read and that information save here for later use.
   */
  float           fw_version[VSAM_NUM_CHANS]; 
} VSAMCNFG;

typedef struct  VSAMCNFG * VSAM_ID;

/* Prototypes */

int  verifyVSAM(short  card,short  channel, char   parm);
int  checkVSAMAi( short channel );
int  checkVSAMBi( short channel, char  parm );
int  checkVSAMBo( short channel );
long VSAM_io_report( char level );
void VSAM_rval_report( short int card,short int flag );
int  VSAM_init( VSAM_ID pcard );
int  VSAM_config( short card, unsigned long addr );
int  VSAM_present( short card,VSAMMEM *pVSAM );
int  VSAM_get_adrs( short card,VSAMMEM **ppVSAM );
int  VSAM_version( short card,unsigned short *pversion );

int bo_VSAM_read(
     short		card,
     short		channel,
     unsigned long	mask,
     unsigned long	*pval
                );

int  getVSAMBitMask(
     short		channel,
     char		spec,
     unsigned long	*pmask
                    );

int  translateVSAMChannel(
     short		channel,
     char		spec,
     VSAMPVT		*ppvt);

int ai_VSAM_read(
     short		card,
     short		channel,
     char		type,
     VSAMPVT		*ppvt,
     float		*prval
               );

int getVSAMRange(
    VSAMMEM		*pMem,
    VSAMPVT		*ppvt,
    float		*pval
                );

int input_VSAM_driver(
   short		card,
   short		channel,
   char		        type,
   unsigned long	mask,
   unsigned long	*pval
                     );

int output_VSAM_driver(
   short		card,
   short		channel,
   unsigned long	mask,
   unsigned long	*pval
                       );

#ifdef __cplusplus
}
#endif  /* __cplusplus */

#endif /* VSAM_H */
