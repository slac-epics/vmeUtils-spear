/* Author:   Marty Kraimer */
/* Date:     17SEP2001     */
/* Changes to work for SIS3302 by Stephanie Allison, 03DEC2012 */

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
#include <epicsThread.h>
#include <epicsExit.h>
#include <epicsExport.h>

#include "ellLib.h"
#include "errlog.h"
#include "devLib.h"
#include "drvSup.h"

#include "drvGtr.h"
#include "SIS3302.h"
/* Register macros not in SIS3302.h */
#define SIS3302_EVENT_CONFIG_READ                0x02000000	  
#define SIS3302_SAMPLE_LENGTH_READ               0x02000004
/* Register value macros not in SIS3302.h */	  
#define SIS3302_CONTROL_STATUS_DISABLE_LED       0x00010000
#define SIS3302_CONTROL_STATUS_ENABLE_LED        0x00000001
#define SIS3302_IRQ_CONTROL_DISABLE              0x00030000
#define SIS3302_IRQ_CONTROL_ENABLE_EOE           0x00000001
#define SIS3302_IRQ_CONTROL_ENABLE_EOL           0x00000002
#define SIS3302_IRQ_CONFIG_ENABLE                0x00000800
#define SIS3302_DAC_CONTROL_SHIFT                0x00000001
#define SIS3302_DAC_CONTROL_LOAD                 0x00000002
#define SIS3302_DAC_CONTROL_CLEAR                0x00000003
#define SIS3302_DAC_CONTROL_BUSY                 0x00008000

#define ARRAYBYTES  0x08000000
#define ARRAYSIZE   ARRAYBYTES/4

#ifdef HAS_IOOPS_H
#include <basicIoOps.h>
#endif

/*
 * Size of local cache
 */
#define DMA_BUFFER_CAPACITY   2048


typedef unsigned int uint32;

#define STATIC static

static char *clockChoices[] = {
    "100 MHz",
    "50 MHz",
    "25 MHz",
    "10 MHz",
    "1 MHz",
    "Random",
    "extClock",
    "2nd 100MHz"
};
static int clockSource[] = { 
    SIS3302_ACQ_SET_CLOCK_TO_100MHZ,
    SIS3302_ACQ_SET_CLOCK_TO_50MHZ,
    SIS3302_ACQ_SET_CLOCK_TO_25MHZ,
    SIS3302_ACQ_SET_CLOCK_TO_10MHZ,
    SIS3302_ACQ_SET_CLOCK_TO_1MHZ,
    SIS3302_ACQ_SET_CLOCK_TO_LEMO_RANDOM_CLOCK_IN,
    SIS3302_ACQ_SET_CLOCK_TO_LEMO_CLOCK_IN,
    SIS3302_ACQ_SET_CLOCK_TO_P2_CLOCK_IN
};

typedef enum {triggerSoft,triggerFPSS,triggerFPSA,triggerAFPS} triggerType;
#define ntriggerChoices 4
static char *triggerChoices[ntriggerChoices] =
{
    "soft","FPstart/FPstop","FPstart/autstop","autstart/FPstop"
};

#define nmultiEventChoices 12
static char *multiEventChoices[nmultiEventChoices] = {
  "2","8","32","128","512","2048","8192","32768","65536","131072","262144","524288"
};
static int multiEventNumber[nmultiEventChoices] = {
  2,8,32,128,512,2048,8192,32768,65536,131072,262144,524288
};

#define npreAverageChoices 8
static char *preAverageChoices[npreAverageChoices] = {
    "1","2","4","8","16","32","64","128"
};

typedef enum { armDisarm, armPostTrigger, armPrePostTrigger } armType;
#define narmChoices 3
static char *armChoices[narmChoices] = {
    "disarm","postTrigger","prePostTrigger"
};

typedef struct sisInfo {
    ELLNODE     node;
    int         card;
    char        *name;
    char        *a32;
	unsigned long	vmeAddrOffst;
    int         intVec;
    int         intLev;
    int         indMultiEventNumber;
    int         nevents;
    int         preAverageChoice;
    armType     arm;
    triggerType trigger;
    int         numberPTS;
    int         numberPPS;
    int         numberPTE;
    gtrhandler usrIH;
    void        *handlerPvt;
    void        *userPvt;
    epicsDmaId  dmaId;
    long        *dmaBuffer;
} sisInfo;

static ELLLIST sisList;
static int sisIsInited = 0;
static int isRebooting;
#ifndef HAS_IOOPS_H
static int sis3302Debug = 0;
#endif

static void writeRegister(sisInfo *psisInfo, int offset,uint32 value)
{
#ifdef HAS_IOOPS_H
    out_be32((volatile void*)(psisInfo->a32 + offset), value);
#else
    char *a32 = psisInfo->a32;
    uint32 *reg;

    if(sis3302Debug >= 2) {
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
                printf("writeRegister: 0x%.8x -> %#x\n", irqBuf[irqOut].value, irqBuf[irqOut].offset);
                if(irqOut == (((sizeof irqBuf)/sizeof irqBuf[0]) - 1))
                    irqOut = 0;
                else
                    irqOut++;
            }
        }
        printf("writeRegister: 0x%.8x -> %#x\n", value, offset);
    }
    reg = (uint32 *)(a32+offset);
    *reg = value;
#endif
}

static uint32 readRegister(sisInfo *psisInfo, int offset)
{
    uint32 value;
#ifdef HAS_IOOPS_H
    value = in_be32((volatile void*)(psisInfo->a32 + offset));
#else
    char *a32 = psisInfo->a32;
    uint32 *reg;

    reg = (uint32 *)(a32+offset);
    value = *reg;
#endif
    return(value);
}

static void sisReboot(void *arg)
{
    sisInfo  *psisInfo;

    isRebooting = 1;
    psisInfo = (sisInfo *)ellFirst(&sisList);
    while(psisInfo) {
        writeRegister(psisInfo,SIS3302_KEY_RESET,1);
        psisInfo = (sisInfo *)ellNext(&psisInfo->node);
    }
}
    
static void initialize()
{
    if(sisIsInited) return;
    sisIsInited=1;
    isRebooting = 0;
    ellInit(&sisList);
   epicsAtExit(sisReboot,NULL);
}

void sisIH(void *arg)
{
    sisInfo *psisInfo = (sisInfo *)arg;
    uint32 value;

    writeRegister(psisInfo,SIS3302_IRQ_CONTROL,SIS3302_IRQ_CONTROL_DISABLE);
    value = readRegister(psisInfo,SIS3302_IRQ_CONTROL);
    if(isRebooting) return;
    switch(psisInfo->arm) {
    case armDisarm:
        break;
    case armPostTrigger:
    case armPrePostTrigger:
        if(psisInfo->usrIH) (*psisInfo->usrIH)(psisInfo->handlerPvt);
        break;
    default:
        epicsInterruptContextMessage("drvSis3302::sisIH Illegal armType\n");
        break;
    }
}

STATIC void readContiguous(sisInfo *psisInfo,
    gtrchannel *phigh,gtrchannel *plow,uint32 *pmemory,
    int nmax,int *nskipHigh, int *nskipLow)
{
}

STATIC void sisinit(gtrPvt pvt)
{
    sisInfo *psisInfo = (sisInfo *)pvt;
    long status;
    uint32 icr;
    
    status = devConnectInterruptVME(psisInfo->intVec,
        sisIH,(void *)psisInfo);
    if(status) {
        errMessage(status,"init devConnectInterrupt failed\n");
        return;
    }
    status = devEnableInterruptLevelVME(psisInfo->intLev);
    if(status) {
        errMessage(status,"init devEnableInterruptLevel failed\n");
    }
    icr = SIS3302_IRQ_CONFIG_ENABLE |
          (psisInfo->intLev <<8) |
          psisInfo->intVec;
    writeRegister(psisInfo,SIS3302_IRQ_CONFIG,icr);
    return;
}

STATIC void sisreport(gtrPvt pvt,int level)
{
    sisInfo *psisInfo = (sisInfo *)pvt;
    uint32 value;

    printf("%s card %d a32 %p intVec %2.2x intLev %d\n",
        psisInfo->name,psisInfo->card,psisInfo->a32,
        psisInfo->intVec,psisInfo->intLev);
    if(level<1) return;
    value = readRegister(psisInfo,SIS3302_CONTROL_STATUS);
    printf("    CSR %8.8x",value);
    value = readRegister(psisInfo,SIS3302_MODID);
    printf(" MODID %8.8x",value);
    value = readRegister(psisInfo,SIS3302_IRQ_CONFIG);
    printf(" INTCONFIG %8.8x",value);
    value = readRegister(psisInfo,SIS3302_IRQ_CONTROL);
    printf(" INTCONTROL %8.8x",value);
    value = readRegister(psisInfo,SIS3302_ACQUISTION_CONTROL);
    printf(" ACQCSR %8.8x",value);
    printf("\n");
    value = readRegister(psisInfo,SIS3302_EVENT_CONFIG_READ);
    printf("   EVENTCONFIG %8.8x",value);
    value = readRegister(psisInfo,SIS3302_START_DELAY);
    printf("  STARTDELAY %u",value);
    value = readRegister(psisInfo,SIS3302_STOP_DELAY);
    printf("  STOPDELAY %u",value);
    value = readRegister(psisInfo,SIS3302_MAX_NOF_EVENT);
    printf(" MAXEVENTS %u",value);
    value = readRegister(psisInfo,SIS3302_ACTUAL_EVENT_COUNTER);
    printf(" EVENTCOUNTER %u",value);
    value = readRegister(psisInfo,SIS3302_SAMPLE_LENGTH_READ);
    printf(" MAXSAMPLES %u",(value&0x1FFFFFC)+4);
    printf("\n");
}

STATIC gtrStatus sisclock(gtrPvt pvt, int value)
{
    sisInfo *psisInfo = (sisInfo *)pvt;
    int clockChoice;
    
    if(value<0 || value>=sizeof(clockChoices)/sizeof(char *))
        return(gtrStatusError);
    clockChoice = clockSource[value];
    /* Reset clock */
    writeRegister(psisInfo,SIS3302_ACQUISTION_CONTROL,
		  SIS3302_ACQ_SET_CLOCK_TO_100MHZ);
    /* Set clock */
    writeRegister(psisInfo,SIS3302_ACQUISTION_CONTROL,clockChoice);
    return(gtrStatusOK);
}

STATIC gtrStatus sistrigger(gtrPvt pvt, int value)
{
    sisInfo *psisInfo = (sisInfo *)pvt;

    if(value<0 || value>=ntriggerChoices) return(gtrStatusError);
    psisInfo->trigger = value;
    return(gtrStatusOK);
}

STATIC gtrStatus sismultiEvent(gtrPvt pvt, int value)
{
    sisInfo *psisInfo = (sisInfo *)pvt;

    if(value<0 || value>=nmultiEventChoices) return(gtrStatusError);
    if(isRebooting) epicsThreadSuspendSelf();
    psisInfo->indMultiEventNumber = value;
    return(gtrStatusOK);
}

STATIC gtrStatus sispreAverage(gtrPvt pvt, int value)
{
    sisInfo *psisInfo = (sisInfo *)pvt;

    if(value<0 || value>=npreAverageChoices) return(gtrStatusError);
    if(isRebooting) epicsThreadSuspendSelf();
    psisInfo->preAverageChoice = value;
    return(gtrStatusOK);
}

STATIC gtrStatus sisnumberPTS(gtrPvt pvt, int value)
{
    sisInfo *psisInfo = (sisInfo *)pvt;

    psisInfo->numberPTS = value;
    return(gtrStatusOK);
}

STATIC gtrStatus sisnumberPPS(gtrPvt pvt, int value)
{
    sisInfo *psisInfo = (sisInfo *)pvt;

    psisInfo->numberPPS = value;
    return(gtrStatusOK);
}
STATIC gtrStatus sisnumberPTE(gtrPvt pvt, int value)
{
    sisInfo *psisInfo = (sisInfo *)pvt;

    psisInfo->numberPTE = value;
    return(gtrStatusOK);
}

STATIC gtrStatus sisarm(gtrPvt pvt, int value)
{
    sisInfo *psisInfo = (sisInfo *)pvt;
    uint32 acr,ecr,elr;
    
    /* Disable all triggers */
    acr = SIS3302_ACQ_DISABLE_LEMO_START_STOP     |
          SIS3302_ACQ_DISABLE_LEMO1_TIMESTAMP_CLR |
          SIS3302_ACQ_DISABLE_INTERNAL_TRIGGER    |
          SIS3302_ACQ_DISABLE_MULTIEVENT          |
          SIS3302_ACQ_DISABLE_AUTOSTART;
    writeRegister(psisInfo,SIS3302_ACQUISTION_CONTROL,acr);
    writeRegister(psisInfo,SIS3302_IRQ_CONTROL,SIS3302_IRQ_CONTROL_DISABLE);
    psisInfo->arm = value;
    if(psisInfo->arm==armDisarm) {
      writeRegister(psisInfo,SIS3302_KEY_DISARM,1);
      writeRegister(psisInfo,SIS3302_CONTROL_STATUS,SIS3302_CONTROL_STATUS_DISABLE_LED);
      return(gtrStatusOK);
    }
    if((psisInfo->arm!=armPostTrigger) && (psisInfo->arm!=armPrePostTrigger)) {
        errlogPrintf("drvSis3302::sisarm Illegal armType\n");
        return(gtrStatusError);
    }
    /* Initialize event config */
    ecr = psisInfo->preAverageChoice << 12;
    ecr |= psisInfo->indMultiEventNumber;
    /* Initialize acquisition config */
    if(psisInfo->trigger != triggerSoft) acr = SIS3302_ACQ_ENABLE_LEMO_START_STOP;
    else                                 acr = 0x000;
    /* Set autostart if desired */
    if(psisInfo->trigger == triggerAFPS) acr |= SIS3302_ACQ_ENABLE_AUTOSTART;
    /* Set # of post-trigger events and enable multi-event mode if needed */
    if (psisInfo->numberPTE > 1) {
        writeRegister(psisInfo,SIS3302_MAX_NOF_EVENT,psisInfo->numberPTE);
	writeRegister(psisInfo,SIS3302_IRQ_CONTROL,SIS3302_IRQ_CONTROL_ENABLE_EOL);
        acr |= SIS3302_ACQ_ENABLE_MULTIEVENT;
	psisInfo->nevents = multiEventNumber[psisInfo->indMultiEventNumber];
    } else {
	writeRegister(psisInfo,SIS3302_MAX_NOF_EVENT,1);
	writeRegister(psisInfo,SIS3302_IRQ_CONTROL,SIS3302_IRQ_CONTROL_ENABLE_EOE);
	psisInfo->nevents = 1;
    }
    /* Set # of post-trigger samples */
    if (psisInfo->numberPTS > 4) elr = psisInfo->numberPTS-4;
    else                         elr = 1;
    elr &= 0x1FFFFFC;
    writeRegister(psisInfo,SIS3302_SAMPLE_LENGTH_ALL_ADC,elr);
    /* Set autostop if desired */
    if((psisInfo->trigger == triggerFPSA) || (psisInfo->trigger == triggerSoft))
        ecr |= EVENT_CONF_ENABLE_SAMPLE_LENGTH_STOP;
    /* If autostart is used or this is prepostTrigger, use wraparound 
       and set STOP_DELAY for post trigger data */
    if((psisInfo->trigger == triggerAFPS) || (psisInfo->arm > armPrePostTrigger)) {
        if (psisInfo->numberPTS > 0)
	    writeRegister(psisInfo,SIS3302_STOP_DELAY,psisInfo->numberPTS);
	else
	    writeRegister(psisInfo,SIS3302_STOP_DELAY,0);
	ecr |= EVENT_CONF_ENABLE_WRAP_PAGE_MODE;
    } else {
	writeRegister(psisInfo,SIS3302_STOP_DELAY,0);
    }
    writeRegister(psisInfo,SIS3302_EVENT_CONFIG_ALL_ADC,ecr);
    writeRegister(psisInfo,SIS3302_ACQUISTION_CONTROL,acr);
    writeRegister(psisInfo,SIS3302_KEY_ARM,1);
    writeRegister(psisInfo,SIS3302_CONTROL_STATUS,SIS3302_CONTROL_STATUS_ENABLE_LED);
    /* Start the acquistion if desired */
    if((psisInfo->trigger == triggerAFPS) || (psisInfo->arm > armPrePostTrigger)) {
        writeRegister(psisInfo,SIS3302_KEY_START,1);
    }
    return(gtrStatusOK);
}

STATIC gtrStatus sissoftTrigger(gtrPvt pvt)
{
    sisInfo *psisInfo = (sisInfo *)pvt;
    writeRegister(psisInfo,SIS3302_KEY_START,1);
    return(gtrStatusOK);
}

STATIC gtrStatus sisreadMemory(gtrPvt pvt,gtrchannel **papgtrchannel)
{
    sisInfo *psisInfo = (sisInfo *)pvt;
    char *pbank;
    int indgroup;
    int numberPPS = psisInfo->numberPPS;

    return(gtrStatusOK);
}

STATIC gtrStatus sisgetLimits(gtrPvt pvt,int16 *rawLow,int16 *rawHigh)
{
    *rawLow = 0;
    *rawHigh = 0xffff;
    return(gtrStatusOK);
}

STATIC gtrStatus sisregisterHandler(gtrPvt pvt,
     gtrhandler usrIH,void *handlerPvt)
{
    sisInfo *psisInfo = (sisInfo *)pvt;
    
    psisInfo->usrIH = usrIH;
    psisInfo->handlerPvt = handlerPvt;
    return(gtrStatusOK);
}

STATIC int sisnumberChannels(gtrPvt pvt)
{
    return(8);
}

STATIC gtrStatus sisclockChoices(gtrPvt pvt,int *number,char ***choice)
{    
    *number = sizeof(clockChoices)/sizeof(char *);
    *choice = clockChoices;
    return(gtrStatusOK);
}

STATIC gtrStatus sisarmChoices(gtrPvt pvt,int *number,char ***choice)
{
    *number = narmChoices;
    *choice = armChoices;
    return(gtrStatusOK);
}

STATIC gtrStatus sistriggerChoices(gtrPvt pvt,int *number,char ***choice)
{
    *number = ntriggerChoices;
    *choice = triggerChoices;
    return(gtrStatusOK);
}

STATIC gtrStatus sismultiEventChoices(gtrPvt pvt,int *number,char ***choice)
{
    *number = nmultiEventChoices;
    *choice = multiEventChoices;
    return(gtrStatusOK);
}

STATIC gtrStatus sispreAverageChoices(gtrPvt pvt,int *number,char ***choice)
{
    *number = npreAverageChoices;
    *choice = preAverageChoices;
    return(gtrStatusOK);
}

STATIC gtrStatus sisname(gtrPvt pvt,char *pname,int maxchars)
{
    sisInfo *psisInfo = (sisInfo *)pvt;
    strncpy(pname,psisInfo->name,maxchars);
    pname[maxchars-1] = 0;
    return(gtrStatusOK);
}

STATIC gtrStatus siswaitDAC(sisInfo *psisInfo)
{
  int cnt = 0;
  for (cnt = 0; cnt < 5000; cnt++) {
    if(!(readRegister(psisInfo,SIS3302_DAC_CONTROL_STATUS) & 
	 SIS3302_DAC_CONTROL_BUSY)) return gtrStatusOK;
  }
  return gtrStatusError;
}
STATIC gtrStatus sisvoltageOffset(gtrPvt pvt, int chan, double value)
{
    sisInfo *psisInfo = (sisInfo *)pvt;
    uint32 dacSelect;
    uint32 dacControl;
    gtrStatus status = gtrStatusOK;

    if ((value > 3.5) || (value < -2.5) || (chan >= 8)) return gtrStatusError;
  /* 
        Input span of the digitizer is 5V.
        DAC offset determines what voltage range this digiter will operate
        e.g. With the dacOffset of 0x0000, range is 1V to 6V
        e.g. With the dacOffset of 0xFFFF, range is -5 to 0V
  */
  dacSelect = (uint32)(65535.0 * (3.5-value)/6.0);
  if (dacSelect > 0xffff) return gtrStatusError;
  writeRegister(psisInfo,SIS3302_DAC_DATA,dacSelect);
  dacControl = (chan<<4)|SIS3302_DAC_CONTROL_SHIFT;
  writeRegister(psisInfo,SIS3302_DAC_CONTROL_STATUS,dacControl);
  if (siswaitDAC(psisInfo)) status = gtrStatusError;
  dacControl = (chan<<4)|SIS3302_DAC_CONTROL_LOAD;
  writeRegister(psisInfo,SIS3302_DAC_CONTROL_STATUS,dacControl);
  if (siswaitDAC(psisInfo)) status = gtrStatusError;
  return status;
}

static gtrops sis3302ops = {
sisinit, 
sisreport, 
sisclock, 
sistrigger,
sismultiEvent,
sispreAverage,
sisnumberPTS,
sisnumberPPS,
sisnumberPTE,
sisarm, 
sissoftTrigger, 
sisreadMemory,
0, /* readRawMemory */
sisgetLimits,
sisregisterHandler,
sisnumberChannels,
0, /*numberRawChannels */
sisclockChoices,
sisarmChoices,
sistriggerChoices,
sismultiEventChoices,
sispreAverageChoices,
sisname,
0, /*setUser*/
0, /*getUser*/
0, /*lock*/
0, /*unlock*/
sisvoltageOffset
};

int sis3302Config(int card,
    unsigned int a32offset,int intVec,int intLev, int useDma)
{
    char *a32;
    gtrops *pgtrops;
    uint32 probeValue = 0;
    sisInfo *psisInfo;
    long status;

    if(!sisIsInited) initialize();
    if(gtrFind(card,&pgtrops)) {
        printf("card is already configured\n");
        return(0);
    }
    if((a32offset & 0x00FFFFFF) != 0) {
        printf("sis3302Config: illegal a32offset (%#x). "
               "Must be multiple of 0x01000000\n", a32offset);
        return(0);
    }
    status = devRegisterAddress("sis3302",atVMEA32,a32offset,0x08000000,(void *)&a32);
    if(status) {
        errMessage(status,"sis3302Config: devRegisterAddress failed\n");
        return(0);
    }
    if(devReadProbe(4,a32+SIS3302_MODID,&probeValue)!=0) {
        printf("sis3302Config: no card at %#x (local address %p)\n",a32offset,(void *)a32);
        printf("sis3302Config probeValue %8.8x\n",probeValue);
        return(0);
    }
    if((probeValue>>16) != 0x3302) {
        printf("Illegal sisType probeValue %8.8x\n",probeValue); return(0);
    }
    psisInfo = calloc(1,sizeof(sisInfo));
    if(!psisInfo) {
        printf("sis3302Config: calloc failed\n");
        return(0);
    }
    psisInfo->card = card;
    psisInfo->name = calloc(1,81);
    strcpy(psisInfo->name,"sis3302");
    psisInfo->a32 = a32;
	psisInfo->vmeAddrOffst = a32offset - (unsigned long)a32; /* remember VMEaddr for DMA */
    psisInfo->intVec = intVec;
    psisInfo->intLev = intLev;
    psisInfo->nevents = 1;
    writeRegister(psisInfo,SIS3302_KEY_RESET,1);
    if(useDma) {
        psisInfo->dmaId = epicsDmaCreate(NULL, NULL);
        if(psisInfo->dmaId == NULL)
            printf("sis3302Config: DMA requested, but not available.\n");
    }
    else {
        psisInfo->dmaId = NULL;
    }
    ellAdd(&sisList,&psisInfo->node);
    gtrRegisterDriver(card,psisInfo->name,&sis3302ops,psisInfo);
    return(0);
}
/*
 * IOC shell command registration
 */
#include <iocsh.h>
static const iocshArg sis3302ConfigArg0 = { "card",iocshArgInt};
static const iocshArg sis3302ConfigArg1 = { "base address",iocshArgInt};
static const iocshArg sis3302ConfigArg2 = { "interrupt vector",iocshArgInt};
static const iocshArg sis3302ConfigArg3 = { "interrupt level",iocshArgInt};
static const iocshArg sis3302ConfigArg4 = { "use DMA",iocshArgInt};
static const iocshArg *sis3302ConfigArgs[] = {
    &sis3302ConfigArg0, &sis3302ConfigArg1, &sis3302ConfigArg2,
    &sis3302ConfigArg3, &sis3302ConfigArg4};
static const iocshFuncDef sis3302ConfigFuncDef =
                      {"sis3302Config",5,sis3302ConfigArgs};
static void sis3302ConfigCallFunc(const iocshArgBuf *args)
{
    sis3302Config(args[0].ival, args[1].ival, args[2].ival,
                 args[3].ival, args[4].ival);
}

/*
 * This routine is called before multitasking has started, so there's
 * no race condition in the test/set of firstTime.
 */
static void
drvSIS3302RegisterCommands(void)
{
    static int firstTime = 1;
    if (firstTime) {
        iocshRegister(&sis3302ConfigFuncDef,sis3302ConfigCallFunc);
        firstTime = 0;
    }
}
epicsExportRegistrar(drvSIS3302RegisterCommands);
