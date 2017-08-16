/*
=============================================================

  Abs:  EPICS Record Support for a VME Module

  Name: vmeCardRecord.c

          Functions in the Record Entry Table
           *  init_record    - Initialize record (two passes)
           *  process        - Process record
           *  cvt_addr       - Populate the DBADDR structure for req field
           *  get_array_info - Supply # of elem and addr of req field.
           *  put_array_info -
           *  get_precision  - Get precision of req field
           *  get_units      - Get the type of engineering unit
           *  get_graphic_double -
           *  get_control_double -
           *  setalarm       - Set record alarm states
           *  monitor        - Post monitors on bptr,val,mstt

       * indicates static function

  Proto: None

  Auth: 18-Jun-2001, K. Luchini       (LUCHINI)
  Rev : dd-mmm-yyyy, Reviewer's Name  (USERNAME)
-------------------------------------------------------------
  Mod:
        dd-mmm-yyyy, First Lastname (USERNAME):
           Comments

=============================================================
*/
#include        "epicsVersion.h"
#if EPICS_VERSION >= 3 && EPICS_REVISION >= 14
#include        "epicsTime.h"
#else
#include        <vxWorks.h>
#include        <types.h>
#include        <vme.h>
#include        <stdioLib.h>
#include        <lstLib.h>
#endif
#include        <string.h>
#include        <stdlib.h>

#include        <dbDefs.h>
#include        <dbAccess.h>
#include        <dbFldTypes.h>
#include        <devSup.h>
#include        <recSup.h>
#include        <dbEvent.h>
#include        <recGbl.h>
#include        <alarm.h>
#include        <errMdef.h> 
#include        <devLib.h>
#define GEN_SIZE_OFFSET
#include        <vmeCardRecord.h>
#undef GEN_SIZE_OFFSET
#include "epicsExport.h"

#ifndef OK
#define OK 0
#endif

/* Local Prototypes */
static void monitor(void *rec_p);
static void setalarm(vmeCardRecord *vme_ps);

/* device support entry table */
typedef struct { 
  long       number;
  DEVSUPFUN  report;
  DEVSUPFUN  init;
  DEVSUPFUN  init_record;
  DEVSUPFUN  get_ioint_info;
  DEVSUPFUN  process;
}vmeCardDset;



/* Create RSET - Record Support Entry Table*/
#define report          NULL
#define initialize      NULL
static long init_record();
static long process();
#define special         NULL
#define get_value       NULL
static long cvt_dbaddr();
static long get_array_info();
static long put_array_info();
static long get_units();
static long get_precision();
#define get_enum_str    NULL
#define get_enum_strs   NULL
#define put_enum_str    NULL
static long get_graphic_double();
static long get_control_double();
#define get_alarm_double   NULL

static int sizeofTypes[] = {0,1,1,2,2,4,4,4,8,2};

#if EPICS_VERSION >= 3 && EPICS_REVISION >= 14 && EPICS_MODIFICATION > 4
#include "epicsExport.h"
#else
struct
#endif
rset vmeCardRSET={
   RSETNUMBER,
   report,
   initialize,
   init_record,
   process,
   special,
   get_value,
   cvt_dbaddr,
   get_array_info,
   put_array_info,
   get_units,
   get_precision,
   get_enum_str,
   get_enum_strs,
   put_enum_str,
   get_graphic_double,
   get_control_double,
   get_alarm_double };

#ifdef DEBUG
int dbg=1;
#endif

epicsExportAddress(rset,vmeCardRSET); 


/*====================================================

  Abs:  Initialize VME Card record

  Name: init_record

  Args: rec_p                     Record info
          Type: struct
          Use:  void *
          Acc:  read-write
          Mech: By reference

        pass                       Invocation flag
          Type: integer
          Use:  int
          Acc:  read-only
          Mech: By value

  Rem: The purpose of this funciton is to verify
       that device support exists for this record type
       and the use it to initialize the record.

  Side: None

  Ret:  long
           OK - Successful operation

=======================================================*/
static long init_record(void *rec_p, int pass)
{
  long              status = OK;
  size_t            bcnt;
  vmeCardRecord    *vme_ps = NULL;
  vmeCardDset      *dset_ps =NULL;
  static char       *taskName_c="vmeCard: init_record";


  /*
   * If this is the first pass we have nothing todo.
   */ 
   vme_ps=(vmeCardRecord *)rec_p;
   if (pass==0) {
     if (vme_ps->nelm<=0)return(status);

     if (vme_ps->ftvl == DBF_STRING) {
        bcnt = MAX_STRING_SIZE;
        vme_ps->bptr = (char *)calloc(vme_ps->nelm,bcnt);
     }else {
       if (vme_ps->ftvl>DBF_ENUM) vme_ps->ftvl=DBF_SHORT;
       bcnt = sizeofTypes[vme_ps->ftvl];
     }
     /* Allocate memory for data buffer */
     vme_ps->bptr = (char *)calloc(vme_ps->nelm,bcnt);
     if ( !vme_ps->bptr ) {
       status = S_dev_noMemory;
       recGblRecordError(status,rec_p,taskName_c);
#ifdef DEBUG
       if ( dbg ) printf("%s failed to allocate memory\n",taskName_c);
#endif
       return(status);
     }
     return(status);
   }

   /* Second pass!
    * Is device support entry table defined?
    * If not, we're in real trouble!
    */
    dset_ps = (vmeCardDset *)(vme_ps->dset);
    if (!dset_ps)
    {
       status = S_dev_noDSET;
       recGblRecordError (status,rec_p,taskName_c);
    }
    /* Does the device support entry table
     * supply the minimum required functions?
     * If not, we're in real trouble!
     */
    else if  ( (dset_ps->number<5) || !dset_ps->init_record )
    {
       status = S_dev_missingSup;
       recGblRecordError(status,rec_p,taskName_c);
    }
    /*
     * Call device support to initialize record
     */
    else
      status=(*dset_ps->init_record)(rec_p);
    return(status);
}



/*====================================================

  Abs: Record processing

  Name: process

  Args: rec_p                     Record info
          Type: struct
          Use:  void *
          Acc:  read-write
          Mech: By reference

  Rem: This function is called by dbProcesses whenever
       it decides that a record should be processed.

  Side: None

  Ret:  long
           OK - Successful operation

=======================================================*/
static long process(void *rec_p)
{
   long           status = OK;
   unsigned char  pact;
   vmeCardRecord *vme_ps = NULL;
   vmeCardDset   *dset_ps=NULL;



   vme_ps = (vmeCardRecord *)rec_p;
   dset_ps = (vmeCardDset *)vme_ps->dset;
   pact = vme_ps->pact;
   if ( !dset_ps || !dset_ps->process )
   {
      /* Leave pact TRUE so that dbProcess doesn't keep calling */
      vme_ps->pact = TRUE;
      status = S_dev_missingSup;
      recGblRecordError(status,rec_p,"vmeCard: process");
      return (status);
   }

  /* Take action
   * If device support sets pact it's starting
   * async processing, so return
   */
   status = (*dset_ps->process)(rec_p);
   if ( !pact && vme_ps->pact )  return (OK);
   vme_ps->pact = TRUE;

  /* Get the timestamp before converting raw
   * data to engineering units.
   */
   recGblGetTimeStamp(rec_p);
   if ( status==OK ) vme_ps->udf = FALSE;

   /* Check for alarms */
   setalarm(vme_ps);

   /* Check event list */
   monitor(rec_p);

   /* Process forward scan link record */
   recGblFwdLink(rec_p);

   vme_ps->pact = FALSE;
   return(status);
}



/*====================================================

  Abs: Reset alarm status and raise alarm monitors

  Name:  monitor

  Args: rec_p                    Record information
          Type: struct
          Use:  caenV265Record *
          Acc:  read-write
          Mech: By reference

  Rem: This function resets the alarm fields NSTA
       and NSEV upon entry to ensure that the alarm
       check starts fresh after processing completes.

       After alarm fields have been reset, alarm monitors
       are raised when a record changes from an alarm
       state to the no alarm state.

       This function also posts monitors on the following
       fields: VAL,OVAL,MSTT,SLST

  Side: None

  Ret:  None

=======================================================*/
static void monitor(void  *rec_p)
{
   unsigned short   monitor_mask;
   vmeCardRecord   *vme_ps = (vmeCardRecord *)rec_p;


   /* get previous stat and sevr  and new stat and sevr*/
   monitor_mask = recGblResetAlarms(rec_p);

   /* 
    * check for value change and then send out monitors
    * connected to the field that have changed
    */ 
   if ( strncmp(vme_ps->val,vme_ps->oval,sizeof(vme_ps->val)!=0) ) {
      /* post events for value change */
      monitor_mask |= DBE_VALUE | DBE_LOG;

      /* update last value monitored */
      strncpy(vme_ps->oval,vme_ps->val,sizeof(vme_ps->val));

      /* post events for value change */
      db_post_events(rec_p,vme_ps->val,monitor_mask);
   }
   
   if (vme_ps->mstt != vme_ps->slst) {
      monitor_mask |= DBE_VALUE | DBE_LOG;
      vme_ps->slst = vme_ps->mstt;
      db_post_events(rec_p,&vme_ps->mstt,monitor_mask);
   }
   if ( vme_ps->nelm ) {
      monitor_mask |= DBE_VALUE | DBE_LOG;
      db_post_events(rec_p,vme_ps->bptr,monitor_mask);
   }
   return;
}


/*====================================================

  Abs: Check alarm conditions of VME Card record

  Name:  setalarm

  Args: vme_ps                   Record information
          Type: struct
          Use:  vmeCardRecord *
          Acc:  read-write
          Mech: By reference

  Rem: This function checks for alarm conditions
       and sets the severity (NSEV) and status (NDTA)
       fields accordingly.

  Side: None

  Ret:  None


=======================================================*/
static void setalarm(vmeCardRecord  *vme_ps)
{
  if ( vme_ps->udf )
     recGblSetSevr((dbCommon *)vme_ps,UDF_ALARM,INVALID_ALARM);
  return;
}


static long cvt_dbaddr(struct dbAddr *addr_ps)
{
    struct vmeCardRecord *vme_ps=NULL;

    vme_ps = (struct vmeCardRecord *)addr_ps->precord;
    addr_ps->pfield = (void *)(vme_ps->bptr);
    addr_ps->no_elements = vme_ps->nelm;
    addr_ps->field_type  = vme_ps->ftvl;
    if (vme_ps->ftvl==DBF_STRING) 
       addr_ps->field_size = MAX_STRING_SIZE;
    else 
       addr_ps->field_size = sizeofTypes[vme_ps->ftvl];
    addr_ps->dbr_field_type = vme_ps->ftvl;
    return(OK);
}

static long get_array_info(struct dbAddr *addr_ps,
                           long          *no_elements_p,
                           long          *offset_p )
{
    struct vmeCardRecord  *vme_ps=NULL;

    vme_ps = (struct vmeCardRecord *)addr_ps->precord;
    *no_elements_p =  vme_ps->nord;
    *offset_p = 0;
    return(OK);
}

static long put_array_info(struct dbAddr *addr_ps,long nNew )
{
    struct vmeCardRecord   *vme_ps=NULL;


    vme_ps = (struct vmeCardRecord *)addr_ps->precord;
    vme_ps->nord = nNew;
    if ( vme_ps->nord > vme_ps->nelm ) 
       vme_ps->nord = vme_ps->nelm;
    return(OK);
}


static long get_units(struct dbAddr *addr_ps,char *units_p)
{
    struct vmeCardRecord       *vme_ps=NULL;


    vme_ps = (struct vmeCardRecord *)addr_ps->precord;
    strncpy(units_p,vme_ps->egu,DB_UNITS_SIZE);
    return(OK);
}



static long get_precision(struct dbAddr *addr_ps,long  *precision_p )
{
    struct vmeCardRecord  *vme_ps = NULL;


    vme_ps = (struct vmeCardRecord *)addr_ps->precord;
    *precision_p = vme_ps->prec;
    if ( addr_ps->pfield!=(void *)vme_ps->bptr )  
      recGblGetPrec(addr_ps,precision_p);
    return(OK);
}

static long get_graphic_double(struct dbAddr       *addr_ps,
                               struct dbr_grDouble *gd_ps )
{
    struct vmeCardRecord     *vme_ps=NULL;



    vme_ps = (struct vmeCardRecord *)addr_ps->precord;
    if (addr_ps->pfield==(void *)vme_ps->bptr) {
        gd_ps->upper_disp_limit = vme_ps->hopr;
        gd_ps->lower_disp_limit = vme_ps->lopr;
    } else recGblGetGraphicDouble(addr_ps,gd_ps);
    return(OK);
}


static long get_control_double(struct dbAddr         *addr_ps,
                               struct dbr_ctrlDouble *cd_ps )
{
    struct vmeCardRecord     *vme_ps=NULL;

    vme_ps = (struct vmeCardRecord *)addr_ps->precord;
    if (addr_ps->pfield==(void *)vme_ps->bptr){
        cd_ps->upper_ctrl_limit = vme_ps->hopr;
        cd_ps->lower_ctrl_limit = vme_ps->lopr;
    } else 
         recGblGetControlDouble(addr_ps,cd_ps);
    return(OK);
}
