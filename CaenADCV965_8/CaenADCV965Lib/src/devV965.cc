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
 * purpose:             EPICS Device Support for CAEN V965_8 VME Charge Integrating Dual Range ADC
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

struct AiV965_8
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
        }devCaenV965_8AI =
                {
                6,
                NULL,
                NULL,
                (DEVSUPFUN)AiV965_8::InitRecord,
                (DEVSUPFUN)AiV965_8::GetIOIntInfo,
                (DEVSUPFUN)AiV965_8::ReadAi,
                NULL
                };

//
//  device( ai, VME_IO,devCaenV965_8AI, "CAEN V965_8")
//
epicsExportAddress( dset, devCaenV965_8AI);

long AiV965_8::
InitRecord( aiRecord *pRec)
        {

        //pRec->linr=0;

        if( drvCaenV965_8Device::recordInit( &pRec->inp, (dbCommon *)pRec) != 0)
                {
                recGblRecordError(S_db_badField, (void *)pRec, (char *)"devCaenV965_8AI (init_record) Illegal INP field");
                return S_db_badField;
                }

        // If we are reading the high register we use this scale
        if( pRec->inp.value.vmeio.parm[0] == 'H')
                pRec->eslo=200e-15;             // menuConvertSLOPE overrides
            else
                pRec->eslo=25e-15;              // menuConvertSLOPE overrides
        return 0;
        }

long AiV965_8::
GetIOIntInfo( int cmd, aiRecord *pRec, IOSCANPVT *ppvt)
        {

        return drvCaenV965_8Device::getIOIntInfo( cmd, &pRec->inp, ppvt);
        }

long AiV965_8::
ReadAi( aiRecord *pRec)
        {
        int rv;

        rv = drvCaenV965_8Device::getValue( &pRec->inp, &pRec->rval);

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
struct LongInV965_8
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
        }devCaenV965_8Longin =
                {
                5,
                NULL,
                NULL,
                (DEVSUPFUN)LongInV965_8::InitRecord,
                (DEVSUPFUN)LongInV965_8::GetIOIntInfo,
                (DEVSUPFUN)LongInV965_8::ReadLongin
                };

//
//  device( longin, VME_IO, devCaenV965_8Longin, "CAEN V965_8")
//
epicsExportAddress( dset, devCaenV965_8Longin);

long LongInV965_8::
InitRecord( longinRecord *pRec)
        {

        if( drvCaenV965_8Device::recordInit( &pRec->inp,(dbCommon *)pRec) != 0)
                {
                recGblRecordError( S_db_badField, (void *)pRec, (char *)"devCaenV965_8Longin (init_record) Illegal INP field");
                return S_db_badField;
                }
        return 0;
        }

long LongInV965_8::
GetIOIntInfo( int cmd, longinRecord *pRec, IOSCANPVT *ppvt)
        {

        return drvCaenV965_8Device::getIOIntInfo( cmd, &pRec->inp, ppvt);
        }

long LongInV965_8::
ReadLongin( longinRecord *pRec)
        {

        return drvCaenV965_8Device::getValue( &pRec->inp, &pRec->val);
        }

////////////////////////////
//                        //
// longout device support //
//                        //
////////////////////////////
struct LongOutV965_8
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
        }devCaenV965_8Longout =
                {
                5,
                NULL,
                NULL,
                (DEVSUPFUN)LongOutV965_8::InitRecord,
                (DEVSUPFUN)LongOutV965_8::GetIOIntInfo,
                (DEVSUPFUN)LongOutV965_8::writeLongout
                };

//
//  device( longout, VME_IO, devCaenV965_8Longout, "CAEN V965_8")
//
epicsExportAddress( dset, devCaenV965_8Longout);

long LongOutV965_8::
InitRecord( longoutRecord *pRec)
        {
        
        if( drvCaenV965_8Device::recordInit( &pRec->out, (dbCommon *)pRec) != 0)
                {
                recGblRecordError( S_db_badField, (void *)pRec, (char *)"devCaenV965_8Longout (init_record) Illegal OUT field");
                return S_db_badField;
                }
        return 0;
        }

long LongOutV965_8::
GetIOIntInfo(int cmd, longoutRecord *pRec, IOSCANPVT *ppvt)
        {

        return drvCaenV965_8Device::getIOIntInfo( cmd, &pRec->out, ppvt);
        }

long LongOutV965_8::
writeLongout(longoutRecord *pRec)
        {

        return drvCaenV965_8Device::putValue( &pRec->out, pRec->val);
        }
