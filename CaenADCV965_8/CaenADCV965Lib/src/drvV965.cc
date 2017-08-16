/*
 * file:                drvV965.cc
 * purpose:             Driver for CAEN V965_8 VME Charge Integrating Dual Range ADC
 * created:             28-Oct-2005
 *                      Oak Ridge National Laboratory
 *
 * revision history:
 *   28-Oct-2005        David H Thompson        initial version
 *   17-Aug-2006        Doug Murray             updated
 */

// This is a driver for the caen v965 VME board.
// DH Thompson 10/28/2005
// DHT@ORNL.GOV


#include <stdio.h>
#include <stdlib.h>

#include <epicsExport.h>
#include <epicsThread.h>
#include <epicsExit.h>
#include <epicsInterrupt.h>
#include <epicsTypes.h>
#include <drvSup.h>
#include <devLib.h>
#include <link.h>

#include "drvV965.h"
#include "drvV965p.h"


// ================== drvCaenV965_8Device Methods ============================
// Constructor = Let every member have a known value initially
drvCaenV965_8Device::
drvCaenV965_8Device()
        {
        // Make sure that every member has a value
        pBoard = NULL;
        event = 0;
        for( int i = 0 ; i < 16; i++)
                for( int j = 0; j < 2; j++) // ADC_HI to ADC_LO
                        {
                        chanData[i][j].event = 0;
                        chanData[i][j].data = 0;
                        chanData[i][j].status = 0;
                        chanData[i][j].threshold = 0;
                        chanData[i][j].gain = 0;
                        }
        int_vector = 0;
        int_level = 0;
        ioscanpvt = NULL;
        numStates = 0;
        currentState = 0;

        //
        // The osi library does not have one of these... (semBCreate)
        // We'll use epicsEventCreate here.
        //
        wakeupCall = epicsEventCreate( epicsEventEmpty);

        for( int i = 0;i < CAEN_NUM_CHAN;i++)
                sampleState[i] = 0;
        }

// Epics device init routine.
// Initialize all configured boards
long drvCaenV965_8Device::
init()
        {
        drvCaenV965_8Device * pvt;
        epicsAtExit( drvCaenV965_8Device::atExit,NULL);
        for( int i = 0;i < NUM_BOARDS;i++)
                {
                if(( pvt = pDevice[i]) == NULL)
                        continue;

                unsigned  vector = pvt->int_vector;
                unsigned  level = pvt->int_level;
                pvt->pBoard->bitSet1.set = drvCaenV965_8Registers::BS1_SoftReset;
                pvt->pBoard->bitSet1.clear = drvCaenV965_8Registers::BS1_SoftReset;
                if( level)
                        {
                        devConnectInterruptVME( vector, (void (*)(void *)) &drvCaenV965_8Device::isr,(void *) pvt);
                        pvt->pBoard->EventTriggerRegister = 1;
                        pvt->pBoard->config( vector,level);
                        devEnableInterruptLevel( intVME,level);
                        }
                pvt->pBoard->show();
                pvt->pBoard->CrateSelect = 0;
                pvt->pBoard->EventTriggerRegister = 1;
                pvt->pBoard->bitSet2.set = drvCaenV965_8Registers::BS2_AllTrig |
                                        drvCaenV965_8Registers::BS2_OverRangeEn |
                                        drvCaenV965_8Registers::BS2_LowThresholdEn |
                                        drvCaenV965_8Registers::BS2_EmptyEnable |
                                        drvCaenV965_8Registers::BS2_SlideEn;
                pvt->pBoard->SlideConstant = 0;
                pvt->pBoard->GeoAddress = 0;
                scanIoInit( &pvt->ioscanpvt);
                }
        return 0;
        }

// Epics report function. Give some info for DBIOR
long drvCaenV965_8Device::
report( int level)
        {
        for( int i = 0;i < NUM_BOARDS;i++)
                if( pDevice[i])
                        {
                        printf( "Board: %d ",i);	
                        if( pDevice[i]->pBoard)
                                {
                                pDevice[i]->pBoard->show();
                                pDevice[i]->show( level);
                                }
                            else
                                printf( "No board found\n");
                        }
        return 0;
        }

// Configure a device object once created.
int drvCaenV965_8Device::
caenV965_8Config( int board, size_t base, int addrSpace, int vector, int level, int states)
        {
        epicsAddressType addrType;
        unsigned long probe;

        if( board < 0 || board >= NUM_BOARDS)
                {
                printf( "caenV965_8Config() Sorry, we don't have storage for board: %d\n", board);
                return -1;
                }

        if(  drvCaenV965_8Device::pDevice[board]!=NULL)
                {
                printf( "caenV965_8Config() Sorry, you have already initialized board: %d\n", board);
                return -1;
                }

        if( addrSpace != 24 && addrSpace != 32)
                {
                printf( "caenV965_8Config() The ADC must be in A24 or A32 space;  using A%d is not recognized\n", addrSpace);
                return -1;
                }

        if( vector >255 || vector < 0)
                {
                printf( "caenV965_8Config() Sorry, the vector for board: %d must be >0 and <256 \n", board);
                return -1;
                }

        if( level > 7 || level < 0)
                {
                printf( "caenV965_8Config() Sorry, the vector for board: %d must be >0 and <256 \n", board);
                return -1;
                }

        // looks good so far:

        drvCaenV965_8Device *pvt = new drvCaenV965_8Device;

        if( addrSpace == 24)
                addrType = atVMEA24;
            else
                addrType = atVMEA32;

        /*
        if( devRegisterAddress( "CaenV965_8", atVMEA24, base, sizeof( drvCaenV965_8Registers), (volatile void **)&pvt->pBoard) != 0)
        */
        if( devRegisterAddress( "CaenV965_8", addrType, base, sizeof( drvCaenV965_8Registers), (volatile void **)&pvt->pBoard) != 0)
                {
                delete pvt;
                return -1;
                }
        if( devReadProbe( sizeof( unsigned long),
                          (volatile const void *)&pvt->pBoard,
                          (void *) &probe) )
                {
                printf( "caenV965_8Config() The board %d at base address %x does not exist\n", board, base);
                (void)devUnregisterAddress( addrType, base, "CaenV965_8");
                delete pvt;
                return -1;
                }
        if( pvt->pBoard->getModelNumber() != drvCaenV965_8Registers::CAEN_MODEL_NUMBER)
                {
                printf( "caenV965_8Config() The board at the given address is not the correct model (Expecting V%d, found V%d)\n", drvCaenV965_8Registers::CAEN_MODEL_NUMBER, pvt->pBoard->getModelNumber());
                (void)devUnregisterAddress( addrType, base, "CaenV965_8");
                delete pvt;
                return -1;
                }

        pDevice[board] = pvt;
        pvt->int_level = level;
        pvt->int_vector = vector;

        for( int i = 0; i < drvCaenV965_8Registers::CAEN_NUM_SIGNALS;i++)
                pvt->pBoard->enableChannel( i, false);

        pvt->numStates = states;
        pvt->currentState = 0;

        return 0;
        }

// This is the interrupt service routine for one board
void drvCaenV965_8Device::
isr( void *pdev)
        {
        drvCaenV965_8Device * pThis=(drvCaenV965_8Device * )pdev;
        unsigned long buffer;
        drvCaenV965_8Registers::OutputBufferWordType type;
        unsigned long eob = 0;
        int chan;
        int range;
        unsigned long event = pThis->event+1;

        while( ! eob)
                {
                buffer = pThis->pBoard->OutputBuffer[0];

                type = (drvCaenV965_8Registers::OutputBufferWordType)( buffer&drvCaenV965_8Registers::OBT_mask);

                switch( type)
                        {

                case drvCaenV965_8Registers::OBT_header:
                        break;

                case drvCaenV965_8Registers::OBT_valid_datum:
                        chan=(buffer & drvCaenV965_8Registers::OBB_CHANNEL)/drvCaenV965_8Registers::OBB_CHANNEL_SHIFT;
                        //chan=(buffer>>18)&15;
                        range=(buffer & drvCaenV965_8Registers::OBB_RG)?ADC_LO:ADC_HI;
                        if( pThis->sampleState[chan] && pThis->currentState == 0)
                                // request to set the threshold;
                                pThis->chanData[chan][range].threshold=(buffer & drvCaenV965_8Registers::OBB_ADC);
                            else
                                if( pThis->sampleState[chan] == 0 || pThis->currentState == pThis->sampleState[chan])
                                        {
                                        pThis->chanData[chan][range].data=(buffer & drvCaenV965_8Registers::OBB_ADC);
                                        pThis->chanData[chan][range].event = event;

                                        if( buffer & drvCaenV965_8Registers::OBB_UN)
                                        pThis->chanData[chan][range].status = 1;
                                        else if( buffer & drvCaenV965_8Registers::OBB_OV)
                                        pThis->chanData[chan][range].status = 2;
                                        else
                                        pThis->chanData[chan][range].status = 0;
                                        }
                        break;

                case drvCaenV965_8Registers::OBT_end_block:
                        // Handle scaniorequest here
                        pThis->currentState++;
                        if( pThis->currentState >= pThis->numStates)
                                {
                                // We must reset for this to work
                                pThis->event = event;
                                pThis->currentState = pThis->numStates;
                                scanIoRequest( pThis->ioscanpvt);

                                epicsEventSignal( pThis->wakeupCall);
                                }
                        // Now, check to see if we are really done.
                        if( pThis->pBoard->StatusRegister2 & drvCaenV965_8Registers::ST2_BufferEmpty)
                                eob = 1;
                        break;

                case drvCaenV965_8Registers::OBT_not_valid_datum:
                default:
                        scanIoRequest( pThis->ioscanpvt);
                        eob = 1;
                        epicsInterruptContextMessage("In drvCaenV965_8Device::isr() - Rec OBT_not_valid_datum\n");
                        break;
                        }
                }
        }

// This gets called when the IOC reboots.
// Try to disable the interrupt.
void drvCaenV965_8Device::
atExit(void *)
        {

        for( int i = 0;i < NUM_BOARDS;i++)
                if( pDevice[i] && pDevice[i]->pBoard)
                        pDevice[i]->pBoard->EventTriggerRegister = 0; // 4.19 = Turns interrupts off

        }

//
// This is a debug routine - Just show what is in the output buffer
//
int drvCaenV965_8Device::
readOutputBuffer()
        {

        for( int i = 0;i < drvCaenV965_8Registers::CAEN_NUM_SIGNALS;i++)
                printf("Chan %d: h=%d/%d l=%d/%d ss=%d\n",
                        i,
                        chanData[i][ADC_HI].data,
                        chanData[i][ADC_HI].status,
                        chanData[i][ADC_LO].data,
                        chanData[i][ADC_LO].status,
                        sampleState[i]);
        return 0;
        }

//
// Print the contents of the Board's ID register
//
void drvCaenV965_8Registers::
show()
        {
        int out;

        printf( "CAEN V965_8 ADC:\n");

        out = (( ROM[BoardIdMSB] & 0xFF) << 16) | (( ROM[BoardId] & 0xFF) << 8) | ( ROM[BoardIdLSB] & 0xFF);
        printf( "%20s: V%d\n", "Model", out);

        printf( "%20s: %d\n", "Version", ROM[Version] & 0xFF);

        out = (( ROM[SerialMSB] & 0xFF) << 8) | ( ROM[SerialLSB] & 0xFF);
        printf( "%20s: %d\n", "Serial Number", out);

        out = (( ROM[OUI_MSB] & 0xFF) << 16) | (( ROM[OUI] & 0xFF) << 8) | ( ROM[OUI_LSB] & 0xFF);
        printf( "%20s: %d [%#08x]\n", "Manufacturer ID", out, out);

        printf( "drvCaenV965_8Registers:: Frmw = 0x%04x OUI = 0x%x ROM OUI = 0x%02x%02x%02x version = 0x%02x Board = 0x%02x%02x%02x Serial = 0x%02x%02x\n",
                (unsigned)FirmwareRevision,
                0xff & OUI,
                0xff & ROM[OUI_MSB],
                0xff & ROM[OUI],
                0xff & ROM[OUI_LSB],
                0xff & ROM[Version],
                0xff & ROM[BoardIdMSB],
                0xff & ROM[BoardId],
                0xff & ROM[BoardIdLSB],
                0xff & ROM[SerialMSB],
                0xff & ROM[SerialLSB]);
        printf("                       level=%d, vec = 0x%02x, iped=%d\n",
                0x07 & InterruptLevel ,
                0xff & InterruptVector,
                0xff & Iped);
        }

/* ============================================================================ *\
** ================== drvCaenV965_8Registers Methods ============================ **
*  These sparce methods operate directly on the hardware                        **
\* ============================================================================ */

int drvCaenV965_8Registers::
config( int vector, int level)
        {

        if( vector < 0 || vector > 255 || level < 1 || level > 7 )
                return -1;

        InterruptVector = vector;
        InterruptLevel = level;
        printf("drvCaenV965_8Registers::config(int %d,int %d)\n",vector,level);
        return 0;
        }

void drvCaenV965_8Device::
show( int level)
        {

        printf("Status 1:%s%s%s%s%s%s\n",
                pBoard->StatusRegister1 & drvCaenV965_8Registers::ST1_Dready       ? " DREADY" : "",
                pBoard->StatusRegister1 & drvCaenV965_8Registers::ST1_GlobalDready ? " Global DREADY" : "",
                pBoard->StatusRegister1 & drvCaenV965_8Registers::ST1_Busy         ? " Busy" : "",
                pBoard->StatusRegister1 & drvCaenV965_8Registers::ST1_GlobalBusy   ? " Global Busy" : "",
                pBoard->StatusRegister1 & drvCaenV965_8Registers::ST1_EvRdy        ? " EvRDY" : "",
                pBoard->StatusRegister1 & drvCaenV965_8Registers::ST1_Purged       ? " Purged" : "");
        if( level > 1)
                readOutputBuffer();
        }

int drvCaenV965_8Device::
recordInit( DBLINK *pLink, dbCommon *pRec)
        {
        int card;
        int rv = 0;
        int signal;
        const char *pparm = "";
        drvCaenV965_8Device *pDev = NULL;

        if( VME_IO != pLink->type)
                return -1;

        card = pLink->value.vmeio.card;

        if( card < 0 || card >= NUM_BOARDS || NULL == ( pDev = pDevice[card]))
                return -1;

        signal = pLink->value.vmeio.signal;

        if( signal < 0 || drvCaenV965_8Registers::CAEN_NUM_SIGNALS < signal)
                return -1;

        if( pLink->value.vmeio.parm)
                pparm = pLink->value.vmeio.parm;

        switch( *pparm)
                {
        case 'I': // IPED 
                break;

        case 'N': // Serial number
                break;

        case 'H': // high ADC
                pDev->pBoard->enableHiChannel( signal, true);
                break;

        case 'L': // Low ADC
                pDev->pBoard->enableLoChannel( signal, true);
                break;

        case 'G': // gain value
        case 'S': // Channel status
        case 'T': // Threshold
        case 'E': // Event number
                break;

        case 0:
                pDev->pBoard->enableChannel( signal, true);
                break;

        case '1' ... '9':
                pDev->sampleState[signal] = atoi( pparm);
                pDev->pBoard->enableChannel( signal, true);
                break;

        default:
                printf("drvCaenV965_8: Invalid option: %c on card %d signal %d\n", *pparm?*pparm:' ',card,signal);
                return -1;
                break;
                }

        return rv;
        }

// ================== drvCaenV965_8DPVT Methods ============================
int drvCaenV965_8Device::
getValue(DBLINK * pLink, epicsInt32 * value) // Returns status
        {

        if( VME_IO != pLink->type)
                return -1;
        int card = pLink->value.vmeio.card;
        int signal = pLink->value.vmeio.signal;
        drvCaenV965_8Device * pDev = NULL;
        if( card < 0 || card >= NUM_BOARDS || NULL == ( pDev= pDevice[card]))
                return -1;

        return pDev->getValue(signal, pLink->value.vmeio.parm, value) ;
        }

int drvCaenV965_8Device::
getValue( int signal, const char * pparm , epicsInt32 * value) // Returns status
        {
        int parm = 0;
        int sparm = 0;
        int rv = 0;
        long tmp;

        if( signal < 0 || drvCaenV965_8Registers::CAEN_NUM_SIGNALS < signal)
                return -1;

	// Grab the field type:
	if( pparm) parm = pparm[0];
	if( parm) sparm = pparm[1];
	

	switch( parm)
                {

	case 'I': // IPED 
		*value = pBoard->getIped();
		break;

	case 'N':
		*value = pBoard->getSerialNumber();
		break;

	case 'H':
		*value = chanData[signal][ADC_HI].data;
		if( chanData[signal][ADC_HI].status&2)
                        rv=-1;
		// Check to see if the data was the latest 
		if( event != chanData[signal][ADC_HI].event)
			rv=-1;
		break;
	
	case 'L':
		*value = chanData[signal][ADC_LO].data;
		if( chanData[signal][ADC_LO].status&2)
                        rv=-1;
		// Check to see if the data was the latest 
		if( event != chanData[signal][ADC_LO].event)
			rv=-1;
		break;

	case 'S':
	        *value = ((chanData[signal][ADC_HI].status&3)<<2) | (chanData[signal][ADC_LO].status&3) ;
		break;

	case 'T':
		*value = chanData[signal][(sparm=='H')?ADC_HI:ADC_LO].threshold;
		break;

	case 'G':
		*value = chanData[signal][ADC_HI].gain;
		break;

        case 'E':
		*value = event;
		break;

	default:
	case 0:
		if( chanData[signal][ADC_LO].status&2 || (chanData[signal][ADC_LO].data > 3840))
                        {
                        tmp = chanData[signal][ADC_HI].data - chanData[signal][ADC_HI].threshold;
                        *value = 8*tmp;
                        if( chanData[signal][ADC_HI].status&2)
                                rv=-1;
                        // Check to see if the data was the latest 
                        if( event != chanData[signal][ADC_LO].event)
                                rv=-1;		    
                        }
		    else
                        {
                        *value = chanData[signal][ADC_LO].data - chanData[signal][ADC_LO].threshold;			
                        // Check to see if the data was the latest 
                        if( event != chanData[signal][ADC_LO].event)
                                rv=-1;
                        }
		break;
                }
	return rv;
        }

int drvCaenV965_8Device::
putValue(DBLINK * pLink, epicsInt32  value) // Return status
        {

        if( VME_IO != pLink->type)
                return -1;
        int card = pLink->value.vmeio.card;
        int signal = pLink->value.vmeio.signal;
        drvCaenV965_8Device * pDev = NULL;

        if( card < 0 || card >= NUM_BOARDS || NULL == ( pDev= pDevice[card]))
                return -1;

        return pDev->putValue(signal,pLink->value.vmeio.parm,value);
        }

int drvCaenV965_8Device::
putValue( int signal , const char * pparm, epicsInt32  value) // Return status
        {

        int rv = 0;
        int parm = 0;
        int sparm = 0;

        if( signal < 0 || drvCaenV965_8Registers::CAEN_NUM_SIGNALS < signal) 
                return -1;

        if( pparm)
                parm = pparm[0];
        if( parm)
                sparm = pparm[1];

        switch( parm)
                {
        case 'I': // IPED 
                pBoard->setIped( value);
                break;

        case 'G':
                chanData[signal][ADC_HI].gain = value;
                break;

        case 'T':
                chanData[signal][( sparm=='H')?ADC_HI:ADC_LO].threshold = value;
                break;

        default:
        case 0:
                rv=-1;
                break;
                }
        return rv;
        }

long drvCaenV965_8Device::
getIOIntInfo( int cmd, DBLINK * pLink, IOSCANPVT * ppvt)
        {
        int rv = 0;

        if( VME_IO != pLink->type)
                return -1;
        int card = pLink->value.vmeio.card;
        drvCaenV965_8Device * pDev = NULL;

        if( card < 0 || card >= NUM_BOARDS || NULL == ( pDev= pDevice[card]))
                return -1;
        int signal = pLink->value.vmeio.signal;
        if( signal < 0 || drvCaenV965_8Registers::CAEN_NUM_SIGNALS < signal)
                return -1;


        *ppvt = pDev->ioscanpvt;

        return rv;
        }

void drvCaenV965_8Device::
setState( int newState)
        {

        if( currentState > 1 && currentState < numStates)
                printf( "drvCaenV965_8Device::setState() currentState < numStates %d %d\n",currentState, numStates);
        currentState = newState;
        }

// ======================= Shell Functions ===============================

// Call this from the vxWorks startup shell to create the board.
extern "C"  int
caenV965_8Config( int board, size_t base, int addrSpace, int vector, int level, int states)
        {

	return drvCaenV965_8Device::caenV965_8Config( board, base, addrSpace, vector, level, states);	
        }

// Handy to find a board
extern "C" int
caenV965_8Probe()
        {
	int i;
        unsigned short probe;
	
	for( i = 0;i < 256;i++)
                if( devReadProbe( sizeof( unsigned short), (volatile const void *)( 0xf0000000 + 0x10000 * i + 0x1000), (void *) &probe) == 0)
			printf( "Found board at %p\n", (void *)( 0x10000 * i + 0x1000));
	return 0;
        }

// Shell level report
int
drvCaenV965_8Report( int level)
        {

	return drvCaenV965_8Device::report( level);
        }
// Reset the state counter
int
drvCaenV965_8SetState( int card , int state)
        {
	drvCaenV965_8Device * pDev = drvCaenV965_8Device::getV965_8Handle( card);

	if( pDev == NULL)
                return -1;
	pDev->setState( state);
	return 0;
        }

// Reset the state counter
int
drvCaenV965_8Wait( int card)
        {
	drvCaenV965_8Device * pDev = drvCaenV965_8Device::getV965_8Handle( card);

	if( pDev == NULL)
                return -1;
	return pDev->wait();
        }

drvCaenV965_8Device *drvCaenV965_8Device::
getV965_8Handle( int card)
        {
	if( card < 0 || card >= NUM_BOARDS )
		return NULL;
        return drvCaenV965_8Device::pDevice[card];
        }

// - Epics required structure for driver level support
struct drvCaenV965_8DSet
        {
        long number;
        DRVSUPFUN report;
        DRVSUPFUN init;
        }drvCaenV965_8 =
        {
        2,
        (DRVSUPFUN) drvCaenV965_8Device::report,
        (DRVSUPFUN) drvCaenV965_8Device::init
        };

// Epics hooks.
// In the dbd file put: driver( drvCaenV965_8)
epicsExportAddress( drvet,drvCaenV965_8);

// This is the static storage for boards;

drvCaenV965_8Device * drvCaenV965_8Device::pDevice[NUM_BOARDS];
