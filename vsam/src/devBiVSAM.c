/* devBiVSAM.c - Device Support Routines for VSAM Binary Input
 *
 *      Author:         Susanna Jacobson
 *      Date:           10-31-97
 */
#include        "epicsVersion.h"
#include        <alarm.h>
#include        <dbDefs.h>         /* DB_MAX_CHOICES    */
#include        <dbAccess.h>       /* for S_db_badField */
#include	<recSup.h>
#include	<devSup.h>
#include        <recGbl.h>
//#if (EPICS_REVISION == 14 && EPICS_MODIFICATION >= 11)
#include  "errlog.h"
//#endif
#include	<biRecord.h>
#include        "VSAM.h"
#include        <epicsExport.h>

/* Local prototypes */
static long init_record(struct biRecord *pbi);
static long read_bi(struct biRecord *pbi);

/* Global variables */
struct {
	long		number;
	DEVSUPFUN	report;
	DEVSUPFUN	init;
	DEVSUPFUN	init_record;
	DEVSUPFUN       get_ioint_info;
	DEVSUPFUN	read_bi;
} devBiVSAM={
	5,
	NULL,
	NULL,
	init_record,
	NULL,
	read_bi};

epicsExportAddress(dset, devBiVSAM);


static long init_record(struct biRecord	*pbi)
{
    unsigned long  mask;
    struct vmeio  *pvmeio;
    int            status = S_db_badField;
    short          card, channel;
    char           bit_spec;
    static char *badField_c = "devBiVSAM (init_record) Illegal INP field";
    static char *badType_c  = "devBiVSAM (init_record) bad type, card, sig, or parm";



    switch (pbi->inp.type) {
      case VME_IO:
	pvmeio   = (struct vmeio *)&(pbi->inp.value);
	card     = pvmeio->card;
	channel  = pvmeio->signal;
	bit_spec = pvmeio->parm[0];
	if (verifyVSAM(card, channel, 'B') >= 0) {
	  if (checkVSAMBi(channel, bit_spec) == OK) {
	    if (getVSAMBitMask(channel, bit_spec, &mask) == OK) {
	      pbi->mask = mask;
	      status = OK;
	    }
	  }
	}
        if ( status!=OK ) 
	  recGblRecordError(status,(void *)pbi,badType_c );
	break;

      default :
	recGblRecordError(status,(void *)pbi,badField_c);
        break;
    }

    return(status);
}

static long read_bi(struct biRecord  *pbi)
{
	unsigned long  value;
	struct vmeio  *pvmeio;
	long          status;
	
	pvmeio = (struct vmeio *)&(pbi->inp.value);
	status = input_VSAM_driver(pvmeio->card,
                                   pvmeio->signal,
                                   pvmeio->parm[0],
                                   pbi->mask,
                                   &value);
        if(status==-1) {
	   status = 2; /* don't convert*/
           if ( recGblSetSevr(pbi,READ_ALARM,INVALID_ALARM) && 
                errVerbose && 
		(pbi->stat!=READ_ALARM || pbi->sevr!=INVALID_ALARM))
	     recGblRecordError(-1,(void *)pbi,"bi_VSAM_read Error");
        }else if(status==-2) {
           status=OK;
           recGblSetSevr(pbi,HW_LIMIT_ALARM,INVALID_ALARM);
        }

        if ( status==OK ) pbi->rval = value;
	return(status);
}

