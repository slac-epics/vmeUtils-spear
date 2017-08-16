/*drvEcdrgcadc.h */

/* Author:   Marty Kraimer; Till Straumann (ecdrgcadc) */
/* Date:     213NOV2001     */

/*************************************************************************
* Copyright (c) 2002 The University of Chicago, as Operator of Argonne
* National Laboratory, and the Regents of the University of California, as
* Operator of Los Alamos National Laboratory. EPICS BASE Versions 3.13.7
* and higher are distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution.
*************************************************************************/

#ifndef drvEcdrgcadcH
#define drvEcdrgcadcH

#ifdef __cplusplus
extern "C" {
#endif

int ecdrgcadcConfig(int card, unsigned int a16offset,
    unsigned int a32offset, int intVec, int intLev, int extra);

#ifdef __cplusplus
}
#endif

#endif /*drvEcdrgcadcH*/
