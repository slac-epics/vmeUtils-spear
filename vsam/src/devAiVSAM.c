/* devAiSIAM.c - Device Support Routines for VSAM Analog Input
 *
 *      Author:         Susanna Jacobson
 *      Date:           10-24-97
 */
#include        "epicsVersion.h"
#include	<string.h>
#include	<stdlib.h>

#include	<alarm.h>
#include	<cvtTable.h>
#include	<dbDefs.h>
#include	<dbAccess.h>
#include	<recSup.h>
#include	<devSup.h>
#include        <recGbl.h>
//#if (EPICS_REVISION == 14 && EPICS_MODIFICATION >= 11)
#include  "errlog.h"
//#endif
#include	<link.h>
#include        <devLib.h>         /* for S_dev_noMemory */
#include	<aiRecord.h>
#include	"VSAM.h"
#include        <epicsExport.h>

/* Local prototypes */
static long init_record(struct aiRecord *pai);
static long read_ai(struct aiRecord *pai);
static long special_linconv(struct aiRecord *pai, int after);
static void aiVSAMconvert(struct aiRecord  *pai, float rval);

/* Global variables */
struct {
	long		number;
	DEVSUPFUN	report;
	DEVSUPFUN	init;
	DEVSUPFUN	init_record;
	DEVSUPFUN	get_ioint_info;
	DEVSUPFUN	read_ai;
	DEVSUPFUN	special_linconv;
} devAiVSAM={
	6,
	NULL,
	NULL,
	init_record,
	NULL,
	read_ai,
	special_linconv};

struct {
	long		number;
	DEVSUPFUN	report;
	DEVSUPFUN	init;
	DEVSUPFUN	init_record;
	DEVSUPFUN       get_ioint_info;
	DEVSUPFUN	read_ai;
} devAiSIAM={
	5,
	NULL,
	NULL,
	init_record,
	NULL,
	read_ai};

epicsExportAddress(dset, devAiSIAM);
epicsExportAddress(dset, devAiVSAM);

static long init_record(struct aiRecord	*pai)
{
	struct vmeio   *pvmeio;
	VSAMPVT        *ppvt;
        long            status = S_db_badField;
        long            iss;
	short           chan;
	char            spec;
        float           slope = 33554431.0;
        static char *badField_c = "devAiVSAM (init_record) Illegal INP field";
        static char *badSig_c = "devAiVSAM (init_record) invalid ai sig field";
        static char *badType_c ="devAiVSAM (init_record) bad type,card,sig or parm field";
        static char *memErr_c = "devAiVSAM (init_record) out of memory";


	/* ai.inp must be a VME_IO */
	switch (pai->inp.type) {
	   case VME_IO:
             /* set linear conversion slope*/
	     pai->eslo = (pai->eguf - pai->egul)/slope; 

             /* Verify that the card is present */
      	     pvmeio = (struct vmeio *)&(pai->inp.value);
     	     chan = pvmeio->signal;
	     spec = pvmeio->parm[0];
	     iss = verifyVSAM(pvmeio->card,chan,spec);
             if ( iss!=OK ) {
               if (iss < 0) 
	         recGblRecordError(status,(void *)pai,badType_c );
	       else 
                 status = OK;	/* card not present */
	     }
             /* Is the channel valid? */
             else if (checkVSAMAi(chan) == OK) {
               /* Setup the private device information */
               ppvt = malloc(sizeof(VSAMPVT));
	       if ((ppvt != NULL) && 
		   (translateVSAMChannel(chan,spec,ppvt)==OK)) {
	         pai->dpvt = ppvt;
                 status = OK;
	       }
               else {
                 status =  S_dev_noMemory;
	         recGblRecordError(status,(void *)pai,memErr_c );
	       }
	     }
             else 
	       recGblRecordError(status,(void *)pai,badSig_c );
	     break;

	   default :
                status = S_db_badField;
		recGblRecordError(status,(void *)pai,badField_c);
	}
	   
	return(status);

}


static long read_ai(struct aiRecord  *pai)
{
	float         value;
	struct vmeio *pvmeio;
	long          status;

	
	pvmeio = (struct vmeio *)&(pai->inp.value);
	status = ai_VSAM_read(pvmeio->card,
                              pvmeio->signal,
                              pvmeio->parm[0],
			      (VSAMPVT *)pai->dpvt,
                              &value);
	if(status==-1) {
	   pai->udf = TRUE;
	   status = 2; /* don't convert*/
	   if ( recGblSetSevr(pai,READ_ALARM,INVALID_ALARM) && 
                errVerbose  && 
                (pai->stat!=READ_ALARM ||pai->sevr!=INVALID_ALARM)) 
	      recGblRecordError(-1,(void *)pai,"ai_VSAM_read Error");
	   return(status); 
	}
        else if(status==-2) {
	   status=OK;
	   recGblSetSevr(pai,HW_LIMIT_ALARM,INVALID_ALARM);
	}
	if(status!=OK) return(status);

	/* Do conversion here because data from VSAM is already a float */
	aiVSAMconvert(pai, value);
	return(2);			/* don't convert */
}

static void aiVSAMconvert(struct aiRecord  *pai, float rval)
{
	double	  val;
	float	  aslo=pai->aslo;
	float	  aoff=pai->aoff;


	val = (double)rval + (double)pai->roff;
	/* adjust slope and offset */
	if(aslo!=0.0) val*=aslo;
	if(aoff!=0.0) val+=aoff;

	/* convert raw to engineering units and signal units */
	if(pai->linr == 0) {
		; /* do nothing*/
	}
	else if(pai->linr == 1) {
	   val = (val * pai->eslo) + pai->egul;
	}
	else { /* must use breakpoint table */
		; /* not implemented */
	}

	/* apply smoothing algorithm */
	if (pai->smoo != 0.0){
	    if (pai->init == TRUE) pai->val = val;	/* initial condition */
	    pai->val = val * (1.00 - pai->smoo) + (pai->val * pai->smoo);
	}else{
	    pai->val = val;
	}
	pai->udf = FALSE;
	return;
}

static long special_linconv(struct aiRecord  *pai,int after)
{
   /* VSAM doc says "The overall dynamic range of the VSAM is 25 bits..." */

	if(!after) return(0);
	/* set linear conversion slope*/
	pai->eslo = (pai->eguf -pai->egul)/33554431.0;
	return(0);
}

