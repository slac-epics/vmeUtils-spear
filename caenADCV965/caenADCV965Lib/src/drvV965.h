// This file defines interfaces avaliable to the vxWorks shell and to record processing.

#include "dbScan.h"
#include "link.h" // from epics base:

#include <epicsEvent.h>
#include <epicsTypes.h>

/******************************************************\
|------------ Normal C interface points ---------------|
\******************************************************/

extern "C"
        {
        // Find a board if possible
        int caenV965Probe();

        // This configures up to 20 boards.
        int caenV965Config( int board, size_t base, int addrSpace, int vector, int level, int states);

        // report function
        int drvCaenV965Report( int level);

        // Reset the state counter
        int drvCaenV965SetState( int board , int state);

        // Reset the state counter
        int drvCaenV965Wait( int board);
        }

// Config:
#define NUM_BOARDS (20)
#define CAEN_NUM_CHAN (16)
enum ADC_RANGE
        {
        ADC_HI = 0,
        ADC_LO = 1
        };


// These are declared in drvV965p.h
class drvCaenV965Registers;

// This is the low level device data structure.
// We create one of these per board.
class drvCaenV965Device
        {

        public:
                drvCaenV965Device();

                /* Initialize the board */

                static long init();
                static long report( int level);
                static int caenV965Config( int board, size_t base, int addrSpace, int vector, int level, int states);

                static void isr( void *pDev);
                static void atExit( void *arg);
                int readOutputBuffer();
                void show( int level);
                unsigned short getIped();
                void setIped( unsigned short);

                static int recordInit( DBLINK *pLink, dbCommon *pRec); // Do any per-record initialization
                int putValue( int signal, const char *parm, epicsInt32 value); // Return status
                static int putValue( DBLINK *pLink, epicsInt32 value); // Return status
                int getValue( int signal, const char *parm , epicsInt32 *value); // Return status
                static int getValue(DBLINK *pLink, epicsInt32  *value); // Return status
                static long getIOIntInfo( int cmd, DBLINK *pLink, IOSCANPVT *ppvt);
                void setState( int newState);
                long wait()
                        {

                        return epicsEventWait( wakeupCall);
                        }

                static drvCaenV965Device *getV965Handle( int board);

        private:

                drvCaenV965Registers *pBoard;
                                        // to end up invalid.

                // Process sync
                IOSCANPVT ioscanpvt;
                epicsEventId wakeupCall; // Set at the same time as scanIoRequest

                unsigned long event; // This is not the event counter on board
                                        // but rather is maintained by the ISR
                unsigned char int_vector;
                unsigned char int_level;
                struct chanData
                        {
                        unsigned short data; // Same as range 0=>high 1=>low
                        unsigned short status; // 1=>under 2=over 0=normal
                        unsigned long event; // A copy of event when ISR last read this value
                        long gain; // Splice the high into the top of the low range - Only high channel
                                        // - 0 = no adjustment
                                        // 4096 is 1 count/count

                        unsigned short threshold; // The zero offset.
                        }chanData[CAEN_NUM_CHAN][2];

                int sampleState[CAEN_NUM_CHAN]; // When state is this then store the data
                int numStates; // Number of triggers per cycle
                int currentState; // Which trigger are we on
                static drvCaenV965Device *pDevice[NUM_BOARDS];     // We process all records on interrupt. All that did not process need
        };
