/* drvVSAM.c - Driver Support Routines for VME Smart Analog Monitor
 *
 *	Author:		Susanna Jacobson
 *	Date:		10-24-97
 */

#include <math.h>         /* for modf() */

#include        "dbDefs.h"
#include        "errMdef.h"        /* errMessage()         */
#include        "devLib.h"         /* devRegisterAddress() */
#include        "errlog.h"         /* epicslogPrintf()     */
#include        "basicIoOps.h"     /* in_be32()            */
#include	"VSAM.h"           /* VSAM_NUM_CHANS,etc   */
#include        "VSAMUtils.h"      /* for VSAM_testMem()   */
#include        "epicsExport.h"
#include        "epicsThread.h"

/* Messages - informational and error */
static char *noCard_c    = "VSAM Card %hd not found at (A24) address 0x%8.8lx\n";
static char *cardFound_c = "VSAM Card %hd found at (A24) address 0x%8.8lx\n";
static char *cardNotFound_c   = "verifyVSAM: card %hd not found\n";
static char *chanOutOfRange   = "verifyVSAM: chan limit %d but chan %d\n";
static char *invParam_c       = "verifyVSAM: unknown param char %c\n";
static char *invRange_c       = "translateVSAMChannel: can't allocate range struct\n";



/* Global varaibles */
int     VSAM_DRV_DEBUG = 0;

/* Local variables */
static VSAM_CARD_LIST  VSAM_card_list;
static short           card_list_inited = 0;
static short           ai_cards_found   = 0;


/* local function prototypes */
static long    init();
static long    report(int level);
static int     VSAM_clear( VSAMMEM *pVSAM );
static int     VSAM_calibrateCheck( VSAMMEM *pMem );
static VSAM_ID VSAM_getByCard( short  card );
static VSAM_ID VSAM_getByAddr( unsigned long baseAddr );

/* Global variables        */
/* VSAM driver entry table */

struct {
	long		number;
	DRVSUPFUN	report;
	DRVSUPFUN	init;
} drvVSAM={2,report,init};
epicsExportAddress(drvet,drvVSAM);


/* Driver initialization routines */

static long init()
{
    int        status=OK;
    VSAM_ID    pcard = NULL;
 

    if( !card_list_inited )  return(OK);

    for( pcard=(VSAM_ID)ellFirst((ELLLIST *)&VSAM_card_list); pcard; pcard = (VSAM_ID)ellNext((ELLNODE *)pcard)) 
    {
       status = VSAM_init( pcard );
       if ( status==OK )  {
	  pcard->present = 1;
          if (VSAM_DRV_DEBUG) 
             printf( "VSAM: card %d initialized successfully at %p (A24)\n\n", pcard->card,pcard->pVSAM );
          ai_cards_found++;
       }
       else {
          printf( "DRVSUP: VSAM card %d found, initialization failed\n", pcard->card);
       } 
    } /* End of i_card FOR loop */  

    return( status );
}


/***************************************************************************************************************************/
/*  Routine: VSAM_config                                                                                                   */
/*                                                                                                                         */
/*  Purpose: Register a new VSAM ADC module                                                                                */
/*                                                                                                                         */
/*  Description:                                                                                                           */
/*                                                                                                                         */
/*  Checks all parameters, then creates a new card structure, initializes it and adds it to the end of the linked list.    */
/*                                                                                                                         */
/*  SYNOPSIS: int VSAM_create(                                                                                             */
/*                  UINT16 cardNo,        Unique Identifier                                                                */
/*                  UINT16 baseAddr,      A24 Base Address )                                                               */
/*  Example:                                                                                                               */
/*            VSAM_config(0,0x40000)                                                                                       */
/***************************************************************************************************************************/

int VSAM_config( short card, unsigned long addr)
{
    int               status=ERROR;
    size_t            len = sizeof(VSAMCNFG); 
    unsigned long     ioBase = 0;
    epicsAddressType  space = atVMEA24;   /* A24/D32 address space */
    char              name_c[40];
    VSAM_ID           pcard = NULL;


    if(!card_list_inited)
    {
        /* Initialize linked list */
        ellInit( (ELLLIST *) &VSAM_card_list);
        card_list_inited = 1;
        if(VSAM_DRV_DEBUG) 
          printf("The size of VSAM Memory Map is %d\n", (int)sizeof(VSAMMEM));
    }


    /* Check card number for duplicate */
    if ( VSAM_getByCard( card ) ) {
        errlogPrintf ("VSAM Create: %hd already existed!\n", card);
        return(status);
    }
    /* Check address for duplicate */
    if ( VSAM_getByAddr(addr) ) {
        errlogPrintf ("VSAM Create: 0x%lx already existed!\n", addr);
        return(status);
    }

    /* Allocate memory for VSAM card */
    pcard = callocMustSucceed(1, len,"VSAM_create");
    sprintf(name_c,"VSAM-%.2hd",card );
    status = devRegisterAddress(name_c, space, addr, sizeof(VSAMMEM),(void *)&pcard->pVSAM);
    if ( status == OK ) 
    {
      pcard->card       = card;
      pcard->bus_addr   = addr;
      ellAdd( (ELLLIST *)&VSAM_card_list, (ELLNODE *)pcard);
      pcard->registered = 1;
      status = OK;
    }
    else
    {
      status =  S_dev_addrMapFail;
      errlogPrintf("VSAM card %d at A24 bus addr=0x%lx local addr=0x%lx is invalid",card,addr,ioBase);
      free(pcard);
    }
    return status;
}

/*****************************************************************/
/* Find VSAM by card number from link list                       */
/*****************************************************************/
static VSAM_ID VSAM_getByCard( short  card )
{
    VSAM_ID  pcard = NULL;

    if( !card_list_inited ) return NULL;

    for(pcard=(VSAM_ID)ellFirst((ELLLIST *)&VSAM_card_list); pcard; pcard = (VSAM_ID)ellNext((ELLNODE *)pcard))
    {
      if ( pcard->card==card ) break;
    }

    return pcard;
}

/****************************************************************/
/* Find VSAM by base address from link list                     */
/****************************************************************/
static VSAM_ID VSAM_getByAddr(unsigned long addr)
{
    VSAM_ID  pcard = NULL;

    if(!card_list_inited)   return NULL;

    for(pcard=(VSAM_ID)ellFirst((ELLLIST *)&VSAM_card_list); pcard; pcard = (VSAM_ID)ellNext((ELLNODE *)pcard))
    {
        if ( pcard->bus_addr==addr ) break;
    }
    return pcard;
}


/*
 * VSAM_present - probe VSAM memory location to 
 * verify if card is in crate. 
 */
int VSAM_present( short card, VSAMMEM *pVSAM )
{ 
     int         status;
     long        found = 0;
     VSAM_ID     pcard = NULL;

      pcard = VSAM_getByCard( card );
      if ( pcard ) {
        status = VSAM_init( pcard );
        if (status==OK)
          found = 1;
     }
     return(found);
}

/*
 * VSAM_init - initialize the VSAM module.
 */
int   VSAM_init( VSAM_ID pcard )
{
     int	     status = OK;
     unsigned long   val;
     short           chan;
     unsigned        len  = sizeof(val);
     VSAMMEM        *pVSAM = NULL;
     volatile uint32_t *ptr=NULL;
     double          nsec  = 0.2;

     if (!pcard) return(ERROR);

     pVSAM = pcard->pVSAM;
     /*    val   = in_be32( (volatile void *)pVSAM );
	   printf("VSAM init: ch0=0x%lx\n",val); */
    
     status = devReadProbe(len,(volatile void *)pVSAM,(void*)&val); 
     if (status) {
        errlogPrintf(noCard_c,(int)pcard->card,(int)pVSAM,0,0,0,0);
        return(status);
     }   
    
     if (VSAM_DRV_DEBUG) {
       errlogPrintf(cardFound_c,(int)pcard->card,(int)pVSAM,0,0,0,0);
     }

    /* Before doing anything, ensure that data and registers 
       are initially zero, because we have seen the case where
       garbage is present in vsam records after a reboot. 
       If a '1' happens to be in the "little endian" bit, we
       know that this will overwrite the CALIB SUCCESS state in
       the status register for as long as the "little endian" bit
       is held high, regardless of the processing of the CALIBRATION
       bit at 1 Hz.                         dayle 29mar2002
     */
     if (VSAM_DRV_DEBUG) { 
       printf("\nBefore the clear\n");
       VSAM_testMem( (const VSAMMEM *)pVSAM );
     }
     VSAM_clear( pVSAM );
     if (VSAM_DRV_DEBUG) { 
        printf("\nAfter the clear\n");
        VSAM_testMem( (const VSAMMEM *)pVSAM );
        printf("\n");
     }

    /* 
     * Set mode to read revision. Allow time for
     * the revision to appear in memory after
     * the mode has been set. Remember that
     * the version number is returned in a
     * channels in floating point format. 
     * NOTE:D1 must be cleared to resume
     * analog processing
     *
     * dayle changed this from '=' to 
     *                           "|=" to not wipe out anything 
     *  already set. 03/28/02 
     *
     */
     val = in_be32((volatile void *)&pVSAM->mode_control); 
     val |= SET_FIRMWARE;
     out_be32((volatile void *)&pVSAM->mode_control,val);


     if (VSAM_DRV_DEBUG)
        printf("Wait of %f seconds before reading fw version\n",nsec);
     epicsThreadSleep(nsec);         
     for ( chan=0,ptr=(volatile uint32_t *)pVSAM->data; chan<VSAM_NUM_CHANS; chan++,ptr++ ) { 
          pcard->fw_version[chan] = in_be32((volatile void *)ptr);
     }

    /* 
     * Reset the MODE CONTROL register to 
     * normal scan, analog data and big-endian mode.
     */
     out_be32((volatile void *)&pVSAM->mode_control,0);
     status = OK;
  
     VSAM_calibrateCheck( pVSAM );
     return( status );
}

static int VSAM_calibrateCheck( VSAMMEM *pVSAM )
{
   int     status = OK;
   int     attempts=0;
   unsigned long val;
   unsigned long calib=0;
   double  nsec = 1.0;

    /* Dayle added check that it's calibrated and if not, wait for it
       on 03/19/02. 
       In all my testing, CALIB SUCCESS was ALWAYS true at this point
       so this code could be removed. dayle 03/29/02
     */     
    /* then the calibration bit is set, so fine */ 
     val = in_be32((volatile void *)&pVSAM->status);
     if ( val & CALIB_SUCCESS ){
        if (VSAM_DRV_DEBUG)  
           printf("Calibration bit is set. Proceed as normal.\n"); 
     }
     else {
       /* then the calibration bit is not set, so retry */
       for (attempts=0; !calib && (attempts<10); attempts++) {
	 val= in_be32((volatile void *)&pVSAM->status);
         if (val &= CALIB_SUCCESS){ 
           printf("Calibration: status register = 0x%lx\n",val);
           calib=1;
           status = OK;
	 }
         epicsThreadSleep(nsec);
       }/* End of FOR statement */ 

       if (!calib) {
         if (VSAM_DRV_DEBUG)
            printf("Waited 10 seconds for calibration to complete and finally gave up.\n");
         status = ERROR;
       }
       else {
         printf("Detected calibration successful after %2d seconds\n",attempts);
       }
     }/* end of if */ 
     return( status );
}


/* Zero all data values iand registers upon initialization */ 
int VSAM_clear( VSAMMEM *pVSAM )
{
    int i;
    volatile uint32_t *ptr=NULL;
   double  nsec = 2.0;

    for (i=0,ptr=(volatile uint32_t *)pVSAM->data; i<32; i++,ptr++) {
      out_be32((volatile void *)ptr,0);
    }

    /* There are 32 range values of type char, but they are accessed via A24/D32 address space */
    for (i=0,ptr=(volatile uint32_t *)pVSAM->range ; i<8; i++,ptr++) {
        out_be32((volatile void *)ptr,0);  
    }

    /* There are 32 ac values of type short, but they are accessed via A24/D32 address space */
    for (i=0,ptr=(volatile uint32_t *)pVSAM->ac; i<16; i++,ptr++) {
        out_be32((volatile void *)ptr,0); 
    }
    /* 2 seconds after a reset valid data is available */
    out_be32((volatile void *)&pVSAM->reset,0);
     if (VSAM_DRV_DEBUG)
       printf("Wait of %f seconds after reset\n",nsec);
     epicsThreadSleep(nsec);

    /* 
     * D0: 0= Normal channel scan, 1= Fast scan mode 
     * D1: 0= Normal analog channel data, 1= Z80 Firmware revision number is present in channel buffer
     * D2: 0= Big endian mode, 1= Little endian mode
     * D3: 0= Internal Calibration failed, 1= Internal calibration successful
     */
    out_be32((volatile void *)&pVSAM->mode_control,0);

    /* Note: can't zero status register because it's read-only */
    out_be32((volatile void *)&pVSAM->pad,0);
    out_be32((volatile void *)&pVSAM->diag_mode,0);
    for (i=0; i<3; i++) {
        out_be32((volatile void *)&pVSAM->padding[i],0);
    } 
    return OK;
}


/* Routines called by init_record() in device support */

/* 
 * verifyVSAM - check validity of card, channel, and other parameters.
 */
int verifyVSAM( short  card, short channel,char   parm)
{
  int        status;
     VSAM_ID    pcard = NULL;


    /* verify that specified card is present */
    pcard = VSAM_getByCard( card );
    if ( pcard && pcard->present ) {
      if ((channel >= VSAM_NUM_CHANS+4) || (channel < 0)) {
	if (VSAM_DRV_DEBUG>1) printf(chanOutOfRange,VSAM_NUM_CHANS,channel);
	status = -2;
      }
      else if ((parm == RANGE_TYPE) || (parm == DATA_TYPE) || (parm == AC_TYPE) || (parm == CSR_TYPE))
      {
	status = OK;
      }
      else {
         status = -2;
	 if (VSAM_DRV_DEBUG) printf(invParam_c,parm);
      }
    }
    else {
      status = 1;
      if (VSAM_DRV_DEBUG) printf(cardNotFound_c,card);
    }
    return(status);
}

/*
 * checkVSAMAi - check that channel specifies a valid analog input.
 *
 * To be called after verifyVSAM() has returned 0.
 *
 * Only the hardware channels (0-31) can be read as floats.
 */
int checkVSAMAi(short channel)
{ 
   if (channel < VSAM_NUM_CHANS) return(0);
   return(-2);
}

/*
 * checkVSAMBi - check that channel and parameter specify a valid binary input.
 *
 * To be called after verifyVSAM() has returned 0.
 *
 * Only the Status Register can be read as binary inputs.
 */
int checkVSAMBi( short channel,char parm )
{
   if (channel != STATUS_CHANNEL) return(-2);
   return(0);
}

/*
 * checkVSAMBo - check that channel and parameter specify a valid binary output.
 *
 * To be called after verifyVSAM() has returned 0.
 *
 * Only the following can be written to:
 *	Reset Register, Mode Control Register, and Test Mode Register.
 */
int checkVSAMBo( short	channel)
{
  if ((channel < VSAM_NUM_CHANS) || 
      (channel == STATUS_CHANNEL))
    return(-2);
  return(0);
}

/*
 * bo_VSAM_read - get initial values for binary outputs and mbbo's.
 */
int bo_VSAM_read( short	        card,
                  short         channel,
                  unsigned long	mask,
                  unsigned long	*pval )
{
    int        status = OK;
    VSAMMEM   *pVSAM = NULL;
    unsigned long val=0;


    status = VSAM_get_adrs( card,&pVSAM );
    if ( status == OK ) {
      if (channel == MODE_CHANNEL) {
	if (pVSAM == 0)  
           status = ERROR;
        else {
	  val   = in_be32((volatile void *)&pVSAM->status);
          *pval = val & mask;
	}
      }
      else {
	*pval = 0;
      }
    }
    return(status);
}

/*
 * getVSAMBitMask - get single bit mask from bit specification.
 *
 * Bit specification is character for bit number: '0', '1', etc.
 * This is only relevant for the Mode Control and Status Registers.
 */
int getVSAMBitMask( short          channel,
                    char           spec,
                    unsigned long *pmask)
{
    int	shift,max_shift;

    if (channel == MODE_CHANNEL) max_shift = 2;
    else if (channel == STATUS_CHANNEL) max_shift = 3;
    else return(0);

    shift = spec - '0';
    if ((shift < 0) || (shift > max_shift)) return(-2);
    *pmask = 1 << shift;
    return(0);
}

/*
 * translateVSAMChannel - get address of 4-byte location containing ch's data.
 *
 * VSAM is D32 only, so byte and two-byte items must be extracted from longs.
 */
int translateVSAMChannel(short    channel,
                         char	  spec,
                         VSAMPVT *ppvt)
{
    short     rem;
    VSAMPVT  *rpvt = NULL;

    switch ((int)spec) {
	case RANGE_TYPE:
	    rem = channel%4;
	    ppvt->lchan = channel-rem;
	    rem = rem*8;
	    ppvt->mask = 0xff << rem;
	    ppvt->shift = rem;
	    break;

	case AC_TYPE:
	    rpvt = malloc(sizeof(VSAMPVT));
	    if (rpvt == NULL) {
	        if (VSAM_DRV_DEBUG) printf( invRange_c );
		return -2;
	    }
	    translateVSAMChannel(channel, RANGE_TYPE, rpvt);
	    ppvt->prange = rpvt;
	    rem = channel%2;
	    ppvt->lchan = channel-rem;
	    rem = rem*16;
	    ppvt->mask = 0xffff << rem;
	    ppvt->shift = rem;
	    break;

	default:
	    ppvt->lchan = channel;
	    ppvt->mask = 0xffffffff;
	    ppvt->shift = 0;
	    break;
    }
    return(0);
}


/* Input and Output routines */

/*
 * ai_VSAM_read - Read floating point value:
 *			analog data or firmware revision number,
 *			value represented by range byte,
 *			or value represented by AC measurement (short).
 */
int ai_VSAM_read( short	    card,
                  short	    channel,
                  char      type,
                  VSAMPVT  *ppvt,
                  float	   *prval)
{
    int              status = OK;
    unsigned long    rlong;
    short	     rshort;
    float	     rfloat;
    double	     dfactor,
	             dpp;
    VSAMMEM         *pVSAM;


    status = VSAM_get_adrs( card,&pVSAM );
    if ( status ==OK ) {

      /* VSAM is D32 only, so bytes and shorts must be extracted here */
      switch ((int)type) {
	case RANGE_TYPE:
	    return(getVSAMRange(pVSAM, ppvt, prval));
	    break;

	case AC_TYPE:
   	    /* AC peak-to-peak voltage is ranges[range]*ac/(2**14)     */
	    /* no AC info unless normal scan and analog data requested */
	    if (pVSAM->status & (FAST_SCAN_MODE|FIRMWARE_REV)) return(-1);

	    /* first get range... */
	    if (getVSAMRange(pVSAM, ppvt->prange, &rfloat) != 0) return(-1);

	    dfactor = (double)rfloat/(double)AC_DIVISOR;

	    /* now get AC measurement */
	    rlong  = in_be32((volatile void *)&pVSAM->ac[ppvt->lchan]);
	    rshort = (short)((rlong & ppvt->mask) >> ppvt->shift);
	    dpp    = dfactor * (double)rshort;
	    *prval = (float)dpp;
	    break;

	default:
	    /* rfloat = (float)in_be32((volatile void *)&pVSAM->data[channel]); */ /* This line is incorrect?  Dereferencing a float as a uint32_t. */
	    rfloat = pVSAM->data[channel]; /* pVSAM->data[channel] is already a float */
	    *prval = rfloat; 
	    break;
      }
    }
    return(status);
}

/*
 * getVSAMRange - derive floating-point range value for channel
 */
int getVSAMRange( VSAMMEM  *pMem,
                  VSAMPVT  *ppvt,
                  float	   *pval)
{
    int                 status = 0;
    unsigned long	rlong,i_range;
    static const float  ranges[] = { 10.24, 5.12, 2.56, 1.28, 0.64, 
                                     0.32,  0.16, 0.08, 0.04, 0.02, 
                                     0.01 };

    rlong = in_be32((volatile void *) &pMem->range[ppvt->lchan] );
    i_range = (char)((rlong & ppvt->mask) >> ppvt->shift);
    if ( i_range>MAX_RANGE_BYTE ) 
      status = -1;
    else
      *pval = ranges[ i_range ];
    return( status );
}

/*
 * input_VSAM_driver - return single- or multi-bit binary value.
 */
int input_VSAM_driver( short           card,
                       short           lchan,
                       char            type,
                       unsigned long   mask,
                       unsigned long  *pval)
{
    int                 status=OK;
    unsigned long	lval = 0;
    VSAMMEM		*pVSAM;


    status = VSAM_get_adrs( card,&pVSAM );
    if ( status ==OK ) {
      if (lchan < VSAM_NUM_CHANS) {
	if (type == RANGE_TYPE) {
	  lval = in_be32((volatile void *)&pVSAM->range[lchan]);
	}
	else if (type == AC_TYPE) {
	  lval = in_be32( (volatile void *)&pVSAM->ac[lchan]);
	}
	else {
	   return(-2);
	}
      }
      else {
	  lval = in_be32((volatile void *)&pVSAM->status);
      }
      *pval = lval & mask;
    }
    return(status);
}

/*
 * output_VSAM_driver - reset VSAM, set test mode, or write to mode control reg.
 */
int output_VSAM_driver( short		card,
                        short		channel,
                        unsigned long	mask,
                        unsigned long	*pval)
{
    int            status=OK;
    unsigned long   sval,
		    lval,
		    rval;
    VSAMMEM         *pVSAM = NULL;


    status = VSAM_get_adrs( card,&pVSAM );
    if ( status ==OK ) {
      switch ((int)channel) {
	case RESET_CHANNEL:
	    out_be32((volatile void *)&pVSAM->reset, 0);
	    break;
	case DIAG_CHANNEL:
	    out_be32((volatile void *)&pVSAM->diag_mode,0);
	    break;
	default:
	    /* Only three bits of mode control register are used */
	    sval = in_be32((volatile void *)&pVSAM->status);
	    sval &= MODE_MASK;
	    rval = *pval;
	    if (mask == MODE_MASK) lval = rval & mask;	/* multi-bit output */
	    else {
		if (rval & mask) lval = sval | mask;	/* set single bit */
		else lval = sval & ~mask;		/* clear single bit */
	    }
	    out_be32((volatile void *)&pVSAM->mode_control,lval);
	    break;
      }
    }
    return(status);
}

/* Driver report routines */

static long report(int	level)
{
    return(VSAM_io_report(level));
}

/*
 * VSAM_io_report - report VSAM's present.  For level 1 report,
 *			print raw data for each channel.
 */
long VSAM_io_report( char level )
{
    VSAM_ID     pcard = NULL;
    VSAMMEM    *pVSAM = NULL;

    if ( !ai_cards_found ) {
        printf("No VSAM Modules present\n");
        return(OK);
    }

    for(pcard=(VSAM_ID)ellFirst((ELLLIST *)&VSAM_card_list); pcard; pcard = (VSAM_ID)ellNext((ELLNODE *)pcard))
    {
      if (level == 0 )
	 printf("VSAM:\tcard %hd\tA24: 0x%06lx\n", pcard->card, (unsigned long)pcard->bus_addr);
       else if (level == 1) 
	  VSAM_rval_report(pcard->card,0);
       else if (level==2)
          VSAM_rval_report(pcard->card,1);
      else {
	pVSAM = pcard->pVSAM;
	 printf("VSAM:\tcard %hd\tA24: %p\t status: 0x%x\n", 
                 pcard->card, 
                 pcard->pVSAM, 
                 in_be32((volatile void *)&pVSAM->status));
      }
    }/* End of FOR loop */
    return OK;
}

/*
 * VSAM_rval_report - report raw analog data, range, and AC measurement
 *		      for each channel.
 * 
 * called by VSAM_io_report() if level is 1
 */
void VSAM_rval_report( short int card, short int flag )
{
    short	      i;                 /* channel index */
    double            val;
    double            version_base=0.0;
    double            version_frac=0.0;
    VSAM_ID           pcard=NULL;
    VSAMMEM           *pVSAM=NULL;
    volatile uint32_t *ptr=NULL;

     pcard = VSAM_getByCard( card );
     if ( pcard && pcard->present ) {
       pVSAM = pcard->pVSAM;
       printf("STATUS reg: 0x%x\n",in_be32((volatile void *)&pVSAM->status)); 
       for (i=0,ptr=(volatile uint32_t *)pVSAM->data; i<VSAM_NUM_CHANS; i++,ptr++)
       {
         if ( flag ) {
           version_frac  = modf((double)pcard->fw_version[i],&version_base); 
           val = (double)in_be32((volatile void *)ptr);
	   printf("\tch %2hd: data %e\t firmware ver: 0x%X\n", 
                  i, 
                  val,
                  (int)version_base);
	}
	 else
	 {
           val = (double)in_be32((volatile void *)ptr);
	   printf("\tch %2hd: data %e\n",i, val);
	 }
      }/* End of Channel FOR loop */
    }/* End of linked list FOR loop */
}


/*
 * VSAM_getAdrs - return the VSAM card base address
 */
int VSAM_get_adrs( short card,VSAMMEM **ppVSAM )
{
    int       status = OK;
    VSAM_ID   pcard = NULL;


    *ppVSAM = NULL;

    /* Get card information */    
    pcard = VSAM_getByCard( card );
    if (pcard && pcard->present )
    {
       *ppVSAM = pcard->pVSAM;
    } 
    else 
    {
      status = ERROR;
    }
    return(status);
}

/*
 * VSAM_version - return the VSAM card firmware veresion
 * 
 *  This function obtains the firmware version number from
 *  a local structure which is populated during the VSAM
 *  initialzation. Therefire, this function should only be 
 *  called after the initialization function has been envoked.
 */
int VSAM_version( short card,unsigned short *pversion )
{
    int      status = OK;
    VSAM_ID  pcard = NULL;

    /* Get card information */    
    pcard = VSAM_getByCard( card );
    if ( !pcard || !pcard->present ) 
        status = ERROR;
    else {
      if ( !pversion ) 
         printf("VSAM Card %hd firmware version %f\n",
                pcard->card,
                pcard->fw_version[0]);
      else 
      {
        *pversion = (unsigned short)pcard->fw_version[0];
      }
    }
    return(status);
}
