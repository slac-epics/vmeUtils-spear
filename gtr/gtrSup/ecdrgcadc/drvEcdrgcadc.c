/* Author:   Marty Kraimer; Till Straumann (ECDRGCADC) */
/* Date:     17SEP2001     */

/*************************************************************************
* Copyright (c) 2002 The University of Chicago, as Operator of Argonne
* National Laboratory, and the Regents of the University of California, as
* Operator of Los Alamos National Laboratory. EPICS BASE Versions 3.13.7
* and higher are distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution.
*************************************************************************/

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <limits.h>

#include <menuFtype.h>
#include <epicsDma.h>
#include <epicsInterrupt.h>
#include <epicsExport.h>

#include "ellLib.h"
#include "errlog.h"
#include "devLib.h"
#include "drvSup.h"

#include "drvGtr.h"
#include "drvEcdrgcadc.h"

#ifdef HAS_IOOPS_H
#include <basicIoOps.h>
#endif

#include <bsp/vmeUniverse.h>

extern unsigned long vmeUniverseReadRegXX();
extern void          vmeUniverseWriteRegXX();
extern int           vmeUniverseSlavePortCfgXX();

#define RDUNI(reg)      vmeUniverseReadRegXX((void*)(pecInfo)->a16,(reg))
#define WRUNI(reg,val)  vmeUniverseWriteRegXX((void*)(pecInfo)->a16,(val),(reg))

#ifndef NumberOf
#define NumberOf(arr) (sizeof(arr)/sizeof(arr[0]))
#endif

/* ECDR memory map register offset 1s  and sizes*/
#define BCSR        0x00000000
/* bits in BCSR */
#define BCSR_BUS_RST		(1<<9)	/* reset local bus */
#define BCSR_CHS_RST		(1<<8)	/* reset channels  */
#define BCSR_CNT_RST		(1<<7)  /* reset packet counter */
#define BCSR_SOFT_SYNC		(1<<5)
#define BCSR_EXT_SYNC_EN	(1<<0)
#define BCSR_INI		(BCSR_CHS_RST|BCSR_CNT_RST|BCSR_EXT_SYNC_EN)

#define INT_ENA_REG 0x00000008
#define INT_SRC_REG 0x0000000c
#define VERSION_REG	0x0000001c
#define RMEM        0x00100000
#define RMEMSEP		0x00080000

#define CHBASE		0x00010000
#define PAIRSEP     0x00010000
#define CHSEP		0x00000010

#define RCSR        0x00000000
#define BURSTCNT	0x00000004

/* bits in RCSR */
#define CHINI		0x8004		/* enable PLL, enable SYNC */
#define RESETBUF	(1<<8)
#define GATE_MODE	0x0008		/* enable gate mode; must not use soft SYNC */


/*
 * Size of local cache
 */
#define DMA_BUFFER_CAPACITY   2048

typedef unsigned int uint32;

#define STATIC static


static char *syncChoices[2] =
{
    "Trigger", "Gate"
};

static char *armChoices[3] = {
    "disarm","postTrigger"
};

typedef struct EcdrgcInfo {
    ELLNODE     node;
    int         card;
    char        *name;
    char        *a32;
	char		*a16;
	unsigned long	vmeAddrOffst;
    int         intVec;
    int         intLev;
    int         intMsk;
    int         armed;
    int         gateModeEna, gateModeEnaCache;
    int         numberPTS;
    gtrhandler usrIH;
    void        *handlerPvt;
    void        *userPvt;
	epicsDmaId	dmaId;
} EcdrgcInfo;

static ELLLIST ecdrList;
static int ecdrIsInited = 0;
int ecdrgcAdcDebug = 0;

static void writeRegister(EcdrgcInfo *pecInfo, int offset, uint32 value)
{
#ifdef HAS_IOOPS_H
    out_be32((volatile void*)(pecInfo->a32 + offset), value);
#else
    char *a32 = pecInfo->a32;
    volatile uint32 *reg;

    if(ecdrgcAdcDebug >= 2) {
        volatile static struct {
            uint32 offset;
            uint32 value;
        } irqBuf[10];
        volatile static int irqIn;
        static int irqOut;

        if(epicsInterruptIsInterruptContext()) {
            irqBuf[irqIn].offset = offset;
            irqBuf[irqIn].value = value;
            if(irqIn == (((sizeof irqBuf)/sizeof irqBuf[0]) - 1))
                irqIn = 0;
            else
                irqIn++;
        }
        else {
            while(irqOut != irqIn) {
                printf("ecdrgcadcirq: 0x%.8x -> %#x\n", irqBuf[irqOut].value, irqBuf[irqOut].offset);
                if(irqOut == (((sizeof irqBuf)/sizeof irqBuf[0]) - 1))
                    irqOut = 0;
                else
                    irqOut++;
            }
        }
        printf("ecdrgcadc: 0x%.8x -> %#x\n", value, offset);
    }
    reg = (volatile uint32 *)(a32+offset);
    *reg = value;
#endif
}

static uint32 readRegister(EcdrgcInfo *pecInfo, int offset)
{
    uint32 value;
#ifdef HAS_IOOPS_H
    value = in_be32((volatile void*)(pecInfo->a32 + offset));
#else
    char *a32 = pecInfo->a32;
    volatile uint32 *reg;

    reg = (volatile uint32 *)(a32+offset);
    value = *reg;
#endif
    return(value);
}

static void pushallRx(EcdrgcInfo *pecInfo, int off, uint32 val)
{
int 	pair;
uint32	reg;
	for ( pair=0, reg = CHBASE+off; pair<4; pair++, reg+=PAIRSEP ) {
		writeRegister(pecInfo, reg,       val);
		writeRegister(pecInfo, reg+CHSEP, val);
	}
}

static void initialize()
{
    if (ecdrIsInited) return;
    ecdrIsInited=1;
    ellInit(&ecdrList);
}

void ecdrIH(void *arg)
{
        EcdrgcInfo *pecInfo = (EcdrgcInfo *)arg;
  
	/* mask */
	WRUNI(UNIV_REGOFF_VINT_EN,   0x0);

	if ( ecdrgcAdcDebug > 0 ) {
		epicsInterruptContextMessage("EcdrgcAdc: Interrupt\n");
	}
	
    /* disable external sync; will be re-enabled when re-armed */
    writeRegister(pecInfo,BCSR,readRegister(pecInfo,BCSR) & ~BCSR_EXT_SYNC_EN);

    switch(pecInfo->armed) {
    case 0:
        break;
    case 1:
        if(pecInfo->usrIH) (*pecInfo->usrIH)(pecInfo->handlerPvt);
        break;
    default:
        epicsInterruptContextMessage("drvEcdrgcadc::ecdrIH Illegal armType\n");
    }
}

#ifdef TODO
STATIC void readContiguous(EcdrgcInfo *pecInfo,
    gtrchannel *phigh,gtrchannel *plow,uint32 *pmemory,
    int nmax,int *nskipHigh, int *nskipLow)
{
    int16 high,low,himask,lomask;
    int ind;

    himask = lomask = pecInfo->psisTypeInfo->dataMask;
    if(pecInfo->trigger == triggerFPGate)
        lomask |= 0x8000;  /* Let G bit through */
    for(ind=0; ind<nmax; ind++) {
        uint32 word;
        if(pecInfo->dmaId) {
            if((pecInfo->dmaBuffer == NULL)
             && ((pecInfo->dmaBuffer = malloc(DMA_BUFFER_CAPACITY*sizeof(epicsUInt32))) == NULL)) {
                printf("No memory for SIS3301 DMA buffer.  Falling back to non-DMA opertaion\n");
                pecInfo->dmaId = NULL;
                word = pmemory[ind];
            }
            else {
                int dmaInd = ind % DMA_BUFFER_CAPACITY;
                if(dmaInd == 0) {
                    unsigned int nnow = nmax - ind;
                    if(nnow > DMA_BUFFER_CAPACITY)
                        nnow = DMA_BUFFER_CAPACITY;
#ifdef EMIT_TIMING_MARKERS
                    writeRegister(pecInfo,CSR,0x00000002);
#endif
                    if(epicsDmaFromVmeAndWait(pecInfo->dmaId,
                                   pecInfo->dmaBuffer,
                                   (unsigned long)(pmemory + ind)+pecInfo->vmeAddrOffst,
                                   VME_AM_EXT_SUP_ASCENDING,
                                   nnow*sizeof(long),
                                   sizeof(long)) != 0) {
                        printf("Can't perform DMA: %s\n", strerror(errno));
                        pecInfo->dmaId = NULL;
                        pecInfo->dmaBuffer[dmaInd] = pmemory[ind];
                    }
#ifdef EMIT_TIMING_MARKERS
                    writeRegister(pecInfo,CSR,0x00020000);
#endif
                }
                word = pecInfo->dmaBuffer[dmaInd];
            }
        }
        else {
            word = pmemory[ind];
        }
        if(*nskipHigh>0) {
            --*nskipHigh;
        } else if(phigh->ndata<phigh->len) {
            high = (word>>16)&himask;
            (phigh->pdata)[phigh->ndata++] = high;
        }
        if(*nskipLow>0) {
            --*nskipLow;
        } else if(plow->ndata<plow->len) {
            low = word&lomask;
            (plow->pdata)[plow->ndata++] = low;
        }
        if((phigh->ndata>=phigh->len) && (plow->ndata>=plow->len)) break;
    }
}
#endif

STATIC void ecdrinit(gtrPvt pvt)
{
    EcdrgcInfo *pecInfo = (EcdrgcInfo *)pvt;
    long status;

    status = devConnectInterrupt(intVME,pecInfo->intVec,
        ecdrIH,(void *)pecInfo);
    if(status) {
        errMessage(status,"init devConnectInterrupt failed\n");
        return;
    }
    status = devEnableInterruptLevel(intVME,pecInfo->intLev);
    if(status) {
        errMessage(status,"init devEnableInterruptLevel failed\n");
    }
    return;
}

STATIC void ecdrreport(gtrPvt pvt,int level)
{
    EcdrgcInfo *pecInfo = (EcdrgcInfo *)pvt;

    printf("%s card %d a32 %p intVec %2.2x intLev %d\n",
        pecInfo->name,pecInfo->card,pecInfo->a32,
        pecInfo->intVec,pecInfo->intLev);
    if(level<1) return;
}


STATIC gtrStatus ecdrtrigger(gtrPvt pvt, int value)
{
    EcdrgcInfo *pecInfo = (EcdrgcInfo *)pvt;

    if(value<0 || value>=NumberOf(syncChoices)) return(gtrStatusError);

    pecInfo->gateModeEna = value;
    return(gtrStatusOK);
}

STATIC gtrStatus ecdrnumberPTS(gtrPvt pvt, int value)
{
    EcdrgcInfo *pecInfo = (EcdrgcInfo *)pvt;

	if ( value < 1 || value > 2*131072 )
		return gtrStatusError;

    pecInfo->numberPTS = value;
    return(gtrStatusOK);
}


STATIC gtrStatus ecdrarm(gtrPvt pvt, int value)
{
unsigned val = CHINI|RESETBUF;
    EcdrgcInfo *pecInfo = (EcdrgcInfo *)pvt;

	/* mask all */
	WRUNI(UNIV_REGOFF_VINT_EN,   0);
    pecInfo->armed = value;
    if( !pecInfo->armed ) {
        writeRegister(pecInfo,BCSR,readRegister(pecInfo,BCSR) & ~BCSR_EXT_SYNC_EN);
    } else {
	pushallRx(pecInfo, BURSTCNT, (pecInfo->numberPTS + 1)/2-1);
	/* reset fifo buffer pointer */
	if ( pecInfo->gateModeEna ) {
		val |= GATE_MODE;
	}
	pecInfo->gateModeEnaCache = pecInfo->gateModeEna;
	pushallRx(pecInfo, RCSR,     val);

	/* clear pending irqs and enable */
	WRUNI(UNIV_REGOFF_VINT_STAT,   0x7ff); /* all LINTs, DMA, VERR, LERR */
	WRUNI(UNIV_REGOFF_VINT_EN,     0x7ff); /* all LINTs, DMA, VERR, LERR */

	if ( pecInfo->gateModeEna ) {
       		writeRegister(pecInfo,BCSR,readRegister(pecInfo,BCSR) | BCSR_EXT_SYNC_EN);
	}
    }

    return(gtrStatusOK);
}

STATIC gtrStatus ecdrsoftTrigger(gtrPvt pvt)
{
    EcdrgcInfo *pecInfo = (EcdrgcInfo *)pvt;
    if ( !pecInfo->armed || pecInfo->gateModeEnaCache )
		return gtrStatusError;
    writeRegister(pecInfo,BCSR,readRegister(pecInfo,BCSR)|BCSR_SOFT_SYNC);
    return(gtrStatusOK);
}

STATIC gtrStatus ecdrreadMemory(gtrPvt pvt,gtrchannel **papgtrchannel)
{
    EcdrgcInfo *pecInfo = (EcdrgcInfo *)pvt;
	unsigned status;

	int 	 errors = 0;
	int 	 i;
	unsigned nelm;
	unsigned long bufaddr;

	while ( (status = RDUNI(UNIV_REGOFF_VINT_STAT) & 0x7ff ) ) {

		if ( status & 0x700 ) {
			if ( status & 0x100 )
				printf("EcdrgcADC (ISR): Spurious DMA IRQ\n");
			if ( status & 0x200 )
				printf("EcdrgcADC (ISR): PCI Bus Error\n");
			if ( status & 0x400 )
				printf("EcdrgcADC (ISR): VME Bus Error\n");
			errors++;
		}

		for ( i=0, bufaddr = (unsigned long)(pecInfo->a32 + RMEM) + pecInfo->vmeAddrOffst; i<8; i++, bufaddr+=RMEMSEP ) {
			if ( !papgtrchannel[i] )
				continue;

			nelm = pecInfo->numberPTS;
			if ( nelm > papgtrchannel[i]->len )
				nelm = papgtrchannel[i]->len;

			if ( epicsDmaFromVmeAndWait(pecInfo->dmaId,
									papgtrchannel[i]->pdata,
									bufaddr,
									VME_AM_EXT_SUP_ASCENDING,
									nelm*sizeof(int16),
									sizeof(long)) != 0) {
				printf("Can't perform DMA: %s\n", strerror(errno));
				papgtrchannel[i]->ndata=0;
				errors++;
				continue;
			}
			papgtrchannel[i]->ndata=nelm;
		}

		WRUNI(UNIV_REGOFF_VINT_STAT, status);
	}
    return(errors ? gtrStatusError : gtrStatusOK);
}

STATIC gtrStatus ecdrgetLimits(gtrPvt pvt,int16 *rawLow,int16 *rawHigh)
{
/*
    EcdrgcInfo *pecInfo = (EcdrgcInfo *)pvt;
*/
    *rawLow  = -32768;
    *rawHigh =  32767;
    return(gtrStatusOK);
}

STATIC gtrStatus ecdrregisterHandler(gtrPvt pvt,
     gtrhandler usrIH,void *handlerPvt)
{
    EcdrgcInfo *pecInfo = (EcdrgcInfo *)pvt;
    
    pecInfo->usrIH = usrIH;
    pecInfo->handlerPvt = handlerPvt;
    return(gtrStatusOK);
}

STATIC int ecdrnumberChannels(gtrPvt pvt)
{
    return(8);
}

STATIC gtrStatus ecdrarmChoices(gtrPvt pvt,int *number,char ***choice)
{
    *number = NumberOf(armChoices);
    *choice = armChoices;
    return(gtrStatusOK);
}

STATIC gtrStatus ecdrtriggerChoices(gtrPvt pvt,int *number,char ***choice)
{
    *number = NumberOf(syncChoices);
    *choice = syncChoices;
    return(gtrStatusOK);
}

STATIC gtrStatus ecdrname(gtrPvt pvt,char *pname,int maxchars)
{
    EcdrgcInfo *pecInfo = (EcdrgcInfo *)pvt;
    strncpy(pname,pecInfo->name,maxchars);
    pname[maxchars-1] = 0;
    return(gtrStatusOK);
}

static gtrops ecdradcops = {
ecdrinit, 
ecdrreport, 
0,  /* clock */
ecdrtrigger,
0,	/* multiEvent */
0,	/* preAvg */
ecdrnumberPTS,
0,	/* numberPTS */
0,  /* numberPTE */
ecdrarm, 
ecdrsoftTrigger, 
ecdrreadMemory,
0,	/* readRawMemory */
ecdrgetLimits,
ecdrregisterHandler,
ecdrnumberChannels,
0,	/* numberRawChannels */
0,	/* clockChoices */
ecdrarmChoices,
ecdrtriggerChoices,
0,	/* multiEventChoices */
0,	/* preAverageChoices */
ecdrname,
0, /*setUser*/
0, /*getUser*/
0, /*lock*/
0, /*unlock*/
0  /*voltageOffset*/
};

int ecdrgcadcConfig(int card, unsigned int a16offset,
    unsigned int a32offset, int intVec, int intLev, int extra)
{
    char name[80];
    char *a32, *a16;
    gtrops *pgtrops;
    uint32 probeValue = 0;
    EcdrgcInfo *pecInfo;
    long status;
	int  i;

    if(!ecdrIsInited) initialize();

	if ( !(intVec & 1) ) {
		fprintf(stderr,"Interrupt vector must be odd\n");
		return -1;
	}
	if ( intLev < 0 || intLev > 7 ) {
		fprintf(stderr,"Invalid interrupt level\n");
		return -1;
	}

    if(gtrFind(card,&pgtrops)) {
        printf("card is already configured\n");
        return(-1);
    }
    if((a32offset&0xFF000000)!=a32offset) {
        printf("ecdrgcadcConfig: illegal a32offset. "
               "Must be multiple of 0x0100000.\n");
        return(-1);
    }
#if 0
	/* devAllocAddress probes every single address -- this can take forever */
    status = devAllocAddress("ecdrgcAdc",atVMEA32,0x01000000,4,(void *)&a32);
#else
    status = devRegisterAddress("ecdrgcAdc",atVMEA32,a32offset,0x01000000,(void *)&a32);
#endif
    if(status) {
        errMessage(status,"ecdrgcadcConfig devRegisterAddress (A32) failed\n");
        return(-1);
    }

	if ((a16offset&0xffff0fff)) {
		printf("ecdrgadcConfig: illegal a16offset. "
			   "Must be a multiple of 0x1000.\n");
		return(-1);
	}
    status = devRegisterAddress("ecdrgcAdc",atVMEA16,a16offset,0x1000,(void *)&a16);
    if(status) {
        errMessage(status,"ecdrgcadcConfig devRegisterAddress (A16) failed\n");
        return(-1);
    }

    if (devReadProbe(4,a16+0,(void*)&probeValue)) {
        printf("ecdrgcadcConfig: no card at %p\n",a16);
        return(-1);
    }
	probeValue = vmeUniverseReadRegXX((void*)a16,0);
    if( 0x000010e3 != probeValue ) {
        printf("No universe chip found at %p; probe value was 0x%08x\n", a16, probeValue);
		return(-1);
    }
    pecInfo = calloc(1,sizeof(EcdrgcInfo));
    if(!pecInfo) {
        printf("ecdrgcadcConfig: calloc failed\n");
        return(-1);
    }
    pecInfo->card = card;
	pecInfo->a16 = a16;
    pecInfo->a32 = a32;
	pecInfo->vmeAddrOffst = a32offset - (unsigned long)a32; /* remember VMEaddr for DMA */
    pecInfo->intVec = intVec;
    pecInfo->intLev = intLev;
    pecInfo->dmaId  = epicsDmaCreate(NULL, NULL);
	if(pecInfo->dmaId == NULL) {
		printf("ecdrgcadcConfig: DMA requested, but not available.\n");
		return(-1);
	}

	/* configure the Universe */
	if ( vmeUniverseSlavePortCfgXX((void*)pecInfo->a16,0,
			VME_AM_EXT_SUP_DATA, a32offset, 0, 0x1000000) ) {
		fprintf(stderr,"Unable to configure target board slave\n");
		return(-1);
	}

    if (devReadProbe(4,a32+VERSION_REG,(void*)&probeValue)) {
        printf("ecdrgcadcConfig: no card at %p\n",a32);
        return(-1);
    }
	printf("Found ECDR-GC-814 at %p; Firmware version 0x%08x\n",a32,probeValue);
	sprintf(name,"ECDR-GC-814 %i (firmware 0x%08x)",card,probeValue);
    pecInfo->name = calloc(1,strlen(name)+1);
    strcpy(pecInfo->name,name);

    /* init BCSR */
    writeRegister(pecInfo, BCSR, BCSR_INI);

	/* enable PCI master */
	WRUNI(UNIV_REGOFF_PCI_CSR, RDUNI(UNIV_REGOFF_PCI_CSR) | 4);

	/* LSB of STATID has a different purpose. The universe
	 * seems to always set LSB on the vector it puts on
	 * the bus...
	 */
	WRUNI(UNIV_REGOFF_VINT_STATID, (intVec &~1)<<24);
	for (i=1; i<8; i++)
		intLev=(intLev<<8)|intLev;

	pecInfo->intMsk = intLev;

	WRUNI(UNIV_REGOFF_VINT_MAP0, intLev & 0x77777777);
	WRUNI(UNIV_REGOFF_VINT_MAP2, intLev & 0x00070777);
	WRUNI(UNIV_REGOFF_VINT_EN,       0);
	/* clear pending bits; just in case... */
	WRUNI(UNIV_REGOFF_VINT_STAT,     0x7ff);

	/* DMA setup */
	WRUNI(UNIV_REGOFF_DCTL,
		UNIV_DCTL_L2V | UNIV_DCTL_VDW_64 | UNIV_DCTL_VAS_A32 | UNIV_DCTL_SUPER
		| UNIV_DCTL_VCT | UNIV_DCTL_LD64EN);
	/* enable interrupts at DMA ctl. */
	WRUNI(UNIV_REGOFF_DGCS, UNIV_DGCS_STATUS_CLEAR | UNIV_DGCS_INT_MSK);

	/* TODO DMA LL SETUP */
	WRUNI(UNIV_REGOFF_DCPP, 0);

	/* still disabled at the universe */
	writeRegister(pecInfo, INT_ENA_REG,    1);
	/* INT on burst acq complete */
	writeRegister(pecInfo, INT_SRC_REG,    0);

	pecInfo->gateModeEnaCache = 0;
	pushallRx(pecInfo, RCSR, CHINI | RESETBUF);

    ellAdd(&ecdrList,&pecInfo->node);
    gtrRegisterDriver(card,pecInfo->name,&ecdradcops,pecInfo);
    return(0);
}

/*
 * IOC shell command registration
 */
#include <iocsh.h>
/* ecdrgcadcConfig(int card,unsigned int a16offset,
   unsigned int a32offset,int intVec,int intLev, int extra) */
static const iocshArg ecdradcConfigArg0 = {"card",iocshArgInt};
static const iocshArg ecdradcConfigArg1 = {"a16offset", iocshArgInt};
static const iocshArg ecdradcConfigArg2 = {"a32offset", iocshArgInt};
static const iocshArg ecdradcConfigArg3 = {"intVec", iocshArgInt};
static const iocshArg ecdradcConfigArg4 = {"intLev", iocshArgInt};
static const iocshArg ecdradcConfigArg5 = {"extra",  iocshArgInt};
static const iocshArg * const ecdradcConfigArgs[6] = {
    &ecdradcConfigArg0, &ecdradcConfigArg1, &ecdradcConfigArg2,
    &ecdradcConfigArg3, &ecdradcConfigArg4, &ecdradcConfigArg5};
static const iocshFuncDef ecdradcConfigFuncDef =
    {"ecdrgcadcConfig",6,ecdradcConfigArgs};
static void ecdradcConfigCallFunc(const iocshArgBuf *arg)
{
    ecdrgcadcConfig(arg[0].ival, arg[1].ival, arg[2].ival, arg[3].ival, 
	          arg[4].ival, arg[5].ival);
}

/*
 * This routine is called before multitasking has started, so there's
 * no race condition in the test/set of firstTime.
 */
static void
drvEcdrgcadcRegisterCommands(void)
{
    static int firstTime = 1;
    if (firstTime) {
        iocshRegister(&ecdradcConfigFuncDef,ecdradcConfigCallFunc);
        firstTime = 0;
    }
}
epicsExportRegistrar(drvEcdrgcadcRegisterCommands);
