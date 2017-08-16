/* devBoVSAM.c - Device Support Routines for VSAM binary output
 *
 *      Author:         Susanna Jacobson
 *      Date:           10-29-97
 */
#include        "epicsVersion.h"
#include        <alarm.h>
#include	<dbDefs.h>
#include        <dbAccess.h>
#include        <recSup.h>
#include	<devSup.h>
#include        <recGbl.h>
//#if (EPICS_REVISION == 14 && EPICS_MODIFICATION >= 11)
#include  "errlog.h"
//#endif
#include	<boRecord.h>
#include	"VSAM.h"
#include        <epicsExport.h>


/* Local prototypes */
static long init_record(struct boRecord *pbo);
static long write_bo(struct boRecord *pbo);

/* Global variables */
struct {
	long		number;
	DEVSUPFUN	report;
	DEVSUPFUN	init;
	DEVSUPFUN	init_record;
	DEVSUPFUN	get_ioint_info;
	DEVSUPFUN	write_bo;
}devBoVSAM={
	5,
	NULL,
	NULL,
	init_record,
	NULL,
	write_bo};

epicsExportAddress(dset, devBoVSAM);


static long init_record(struct boRecord	*pbo)
{
    unsigned long  value, mask;
    struct vmeio  *pvmeio;
    int            status = S_db_badField;
    short          card, channel;
    char           bit_spec;
    static char *badField_c = "devBoVSAM (init_record) Illegal OUT field";
    static char *badType_c =  "devBoVSAM (init_record) bad type, card, sig, or parm";


    switch (pbo->out.type) {
    case VME_IO:
	pvmeio = (struct vmeio *)&(pbo->out.value);
	card = pvmeio->card;
	channel = pvmeio->signal;
	bit_spec = pvmeio->parm[0];
	if (verifyVSAM(card, channel, 'B') >= 0) {
	  if (checkVSAMBo(channel) == OK) {
	    if (getVSAMBitMask(channel, bit_spec, &mask) == OK) {
	      pbo->mask = mask;
	      status = bo_VSAM_read(card,channel,mask,&value);
              if(status == OK) 
                pbo->rbv = pbo->rval = value;
              else 
                status = 2;
	    }
	  }
	}
        if ((status!=OK) && (status!=2))
	  recGblRecordError(S_db_badField,(void *)pbo,badType_c );
	break;

      default :
	recGblRecordError(status,(void *)pbo,badField_c );
	break;
    }
    return(status);
}

static long write_bo(struct boRecord	*pbo)
{
    struct vmeio *pvmeio;
    int	          status;

	
    pvmeio = (struct vmeio *)&(pbo->out.value);
    status = output_VSAM_driver(pvmeio->card, 
                                pvmeio->signal,
                                pbo->mask,
                                &pbo->rval);
    if(status!=OK) {
    	if ( recGblSetSevr(pbo,WRITE_ALARM,INVALID_ALARM) && 
             errVerbose &&
	     (pbo->stat!=WRITE_ALARM || pbo->sevr!=INVALID_ALARM))
	  recGblRecordError(-1,(void *)pbo,"output_VSAM_driver Error");
    }
    return(OK);
}
