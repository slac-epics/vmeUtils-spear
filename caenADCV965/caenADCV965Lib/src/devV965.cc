#include <longoutRecord.h>
#include <longinRecord.h>
#include <aiRecord.h>
#include <dbAccess.h>                           /* For S_db_badField constant macro */
#include <devSup.h>                             /* For device support entry table declarations */
#include <recGbl.h> 
#       ifndef EPICS_313
#       include <epicsExport.h> /* 3.14.4 Port */
#       endif

#include "drvV965.h"

/*
 * file:                devV965.cc
 * purpose:             EPICS Device Support for CAEN V965 VME Charge Integrating Dual Range ADC
 * created:             28-Oct-2005
 *                      Oak Ridge National Laboratory
 *
 * revision history:
 *   28-Oct-2005        David H Thompson        initial version
 *   17-Aug-2006        Doug Murray             updated
 */

///////////////////////////
//                       //
// ai device support     //
//                       //
///////////////////////////

struct AiDset
        {
        /*
         * analog input dset
         */
        long number;
        DEVSUPFUN dev_report;
        DEVSUPFUN init;
        DEVSUPFUN init_record;                  /*returns: (-1,0)=>(failure,success)*/
        DEVSUPFUN get_ioint_info;
        DEVSUPFUN read_ai;                      /*(0,2)=> success and convert,don't convert)*/
                                                /* if convert then raw value stored in rval */
        DEVSUPFUN special_linconv;
                                                /* Since this is c++ I can try this packagin scheme. */
        static long InitRecord( aiRecord *pRec);
        static long GetIOIntInfo( int cmd, aiRecord *pRec, IOSCANPVT *ppvt);
        static long ReadAi( aiRecord *pRec);
        }devCaenV965AI =
                {
                6,
                NULL,
                NULL,
                (DEVSUPFUN)AiDset::InitRecord,
                (DEVSUPFUN)AiDset::GetIOIntInfo,
                (DEVSUPFUN)AiDset::ReadAi,
                NULL
                };

//
//  device( ai, VME_IO,devCaenV965AI, "CAEN V965")
//
epicsExportAddress( dset, devCaenV965AI);

long AiDset::
InitRecord( aiRecord *pRec)
        {

        //pRec->linr=0;

        if( drvCaenV965Device::recordInit( &pRec->inp, (dbCommon *)pRec) != 0)
                {
                recGblRecordError(S_db_badField, (void *)pRec, (char *)"devCaenV965AI (init_record) Illegal INP field");
                return S_db_badField;
                }

        // If we are reading the high register we use this scale
        if( pRec->inp.value.vmeio.parm[0] == 'H')
                pRec->eslo=200e-15;             // menuConvertSLOPE overrides
            else
                pRec->eslo=25e-15;              // menuConvertSLOPE overrides
        return 0;
        }

long AiDset::
GetIOIntInfo( int cmd, aiRecord *pRec, IOSCANPVT *ppvt)
        {

        return drvCaenV965Device::getIOIntInfo( cmd, &pRec->inp, ppvt);
        }

long AiDset::
ReadAi( aiRecord *pRec)
        {
        int rv;

        rv = drvCaenV965Device::getValue( &pRec->inp, &pRec->rval);

        // if( pRec->linr == menuConvertNO_CONVERSION)
        //        {
        //        pRec->val=pRec->rval;
        //        if( rv == OK)
        //              rv=2;
        //        }
        return rv; // dont convert 
        }

///////////////////////////
//                       //
// longin device support //
//                       //
///////////////////////////
struct LongInDset
        {
        long number;
        DEVSUPFUN dev_report;
        DEVSUPFUN init;
        DEVSUPFUN init_record;                  /* returns: (-1,0)=>(failure,success)*/
        DEVSUPFUN get_ioint_info;
        DEVSUPFUN read_longin;                  /* returns: (-1,0)=>(failure,success)*/
        static long InitRecord( longinRecord *pAI);
        static long GetIOIntInfo( int cmd, longinRecord *pAI, IOSCANPVT *ppvt);
        static long ReadLongin( longinRecord *pAI);
        }devCaenV965Longin =
                {
                5,
                NULL,
                NULL,
                (DEVSUPFUN)LongInDset::InitRecord,
                (DEVSUPFUN)LongInDset::GetIOIntInfo,
                (DEVSUPFUN)LongInDset::ReadLongin
                };

//
//  device( longin, VME_IO, devCaenV965Longin, "CAEN V965")
//
epicsExportAddress( dset, devCaenV965Longin);

long LongInDset::
InitRecord( longinRecord *pRec)
        {

        if( drvCaenV965Device::recordInit( &pRec->inp,(dbCommon *)pRec) != 0)
                {
                recGblRecordError( S_db_badField, (void *)pRec, (char *)"devCaenV965Longin (init_record) Illegal INP field");
                return S_db_badField;
                }
        return 0;
        }

long LongInDset::
GetIOIntInfo( int cmd, longinRecord *pRec, IOSCANPVT *ppvt)
        {

        return drvCaenV965Device::getIOIntInfo( cmd, &pRec->inp, ppvt);
        }

long LongInDset::
ReadLongin( longinRecord *pRec)
        {

        return drvCaenV965Device::getValue( &pRec->inp, &pRec->val);
        }

////////////////////////////
//                        //
// longout device support //
//                        //
////////////////////////////
struct LongOutDset
        {
        long number;
        DEVSUPFUN dev_report;
        DEVSUPFUN init;
        DEVSUPFUN init_record;                  /*returns: (-1,0)=>(failure,success) */
        DEVSUPFUN get_ioint_info;
        DEVSUPFUN write_longout;                /* (-1,0)=>(failure,success */
        static long InitRecord(longoutRecord *pRec);
        static long GetIOIntInfo(int cmd, longoutRecord *pRec, IOSCANPVT *ppvt);
        static long writeLongout(longoutRecord *pRec);
        }devCaenV965Longout =
                {
                5,
                NULL,
                NULL,
                (DEVSUPFUN)LongOutDset::InitRecord,
                (DEVSUPFUN)LongOutDset::GetIOIntInfo,
                (DEVSUPFUN)LongOutDset::writeLongout
                };

//
//  device( longout, VME_IO, devCaenV965Longout, "CAEN V965")
//
epicsExportAddress( dset, devCaenV965Longout);

long LongOutDset::
InitRecord( longoutRecord *pRec)
        {
        
        if( drvCaenV965Device::recordInit( &pRec->out, (dbCommon *)pRec) != 0)
                {
                recGblRecordError( S_db_badField, (void *)pRec, (char *)"devCaenV965Longout (init_record) Illegal OUT field");
                return S_db_badField;
                }
        return 0;
        }

long LongOutDset::
GetIOIntInfo(int cmd, longoutRecord *pRec, IOSCANPVT *ppvt)
        {

        return drvCaenV965Device::getIOIntInfo( cmd, &pRec->out, ppvt);
        }

long LongOutDset::
writeLongout(longoutRecord *pRec)
        {

        return drvCaenV965Device::putValue( &pRec->out, pRec->val);
        }
