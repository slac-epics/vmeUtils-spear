/* $Id: iocMonInit.c,v 1.3 2004/08/18 00:36:10 saa Exp $ */

/* Author: Till Straumann <strauman@slac.stanford.edu>, 2003 */

/* A driver module with the sole purpose of initializing the iocMon library
 * we use 'dbd' magic to enforce linking and use the 'init' method to let
 * iocInit initialize the library without intervention from the startup
 * script.
 */
#include "drvSup.h"
#include "epicsExport.h"

/* static long report(int level); */

static long init();

struct drvet iocMonInit = {
    2,
    0,
    (DRVSUPFUN) init
};
epicsExportAddress(drvet, iocMonInit);

extern int ppcCpuTempInit(int forceThrm3Init);
extern int iocMonSvgmBspSupportInit();

static long init()
{
int rval = 0;
#ifdef __PPC__
	rval |= ppcCpuTempInit(0);
#endif
#ifdef __rtems__
	rval |= iocMonSvgmBspSupportInit();
#endif
	return rval;
}
