// This is a driver for the caen v965 VME board.
// DH Thompson 10/28/2005
// DHT@ORNL.GOV

// This is a driver for the caen v965 VME board.
// DH Thompson 10/28/2005
// DHT@ORNL.GOV
#include "dbScan.h"
#include <libcpu/io.h>

#include "epicsMutex.h"

/*
 * file:                drvV965p.h
 * purpose:             Driver for CAEN V965 VME Charge Integrating Dual Range ADC
 * created:             28-Oct-2005
 *                      Oak Ridge National Laboratory
 *
 * revision history:
 *   28-Oct-2005        David H Thompson        initial version
 *   17-Aug-2006        Doug Murray             updated
 */
class drvCaenV965Device;

class drvCaenV965Registers
        {
        // We will allow this class to access the registers directly where it makes sense.
        friend class drvCaenV965Device;

        public:
                static const int CAEN_NUM_SIGNALS = 16;
                static const int CAEN_MODEL_NUMBER = 965;

                void show();
                inline int getModelNumber();
                inline long getSerialNumber();
                inline unsigned short getIped();
                inline void setIped( unsigned short val);
                inline int config( int vector, int level);
                inline void enableChannel( int chan, bool enable);
                inline void enableLoChannel( int chan, bool enable);
                inline void enableHiChannel( int chan, bool enable);

        private:
                drvCaenV965Registers()
                        {
                        }; // No public constructor

                class CaenD16
                        {
                        public:
                                inline operator unsigned short();
                                inline unsigned short operator =( const unsigned short data);

                        private:
                                unsigned short loc;
                        };

                class CaenD32
                        {
                        public:
                                inline operator unsigned long();
                                inline unsigned long operator =( const unsigned long data);

                        private:
                                unsigned long loc;
                        };

                struct bitSet
                        {
                        CaenD16 set;
                        CaenD16 clear;
                        };

                struct ThresholdPair
                        {
                        CaenD16 high;
                        CaenD16 low;
                        };

                //
                // Data:
                // Registers: Reference page 33 of the manual
                //
                CaenD32 OutputBuffer[0x200];            // All are the same
                unsigned char unused_0[0x800];          // 0x800
                CaenD16 FirmwareRevision;               // 0x1000
                CaenD16 GeoAddress;                     // 0x1002
                CaenD16 MSCT_CBLT_Address;              // 0x1004
                bitSet bitSet1;                         // 0x1006
                CaenD16 InterruptLevel;                 // 0x100a
                CaenD16 InterruptVector;                // 0x100c
                CaenD16 StatusRegister1;                // 0x100e
                CaenD16 ControlRegister1;               // 0x1010
                CaenD16 ADERHigh;                       // 0x1012
                CaenD16 ADERLow;                        // 0x1014
                CaenD16 SingleShotReset;                // 0x1016
                unsigned char unused1[2];               // 0x1018
                CaenD16 MCST_CBLTCtrl;                  // 0x101a
                unsigned char unused2[4];               // 0x101c
                CaenD16 EventTriggerRegister;           // 0x1020
                CaenD16 StatusRegister2;                // 0x1022
                CaenD16 EventCounterL;                  // 0x1024
                CaenD16 EventCounterH;                  // 0x1026
                CaenD16 IncrementEvent;                 // 0x1028
                CaenD16 IncrementOffset;                // 0x102a
                CaenD16 LoadTestRegister;               // 0x102c
                CaenD16 FCLRWindow;                     // 0x102e
                unsigned char unused3[2];               // 0x1030
                bitSet bitSet2;                         // 0x1032
                CaenD16 WMemoryTestAddress;             // 0x1036
                CaenD16 MemoryTestWord_High;            // 0x1038
                CaenD16 MemoryTestWord_Low;             // 0x103a
                CaenD16 CrateSelect;                    // 0x103c
                CaenD16 TestEventWrite;                 // 0x103e
                CaenD16 EventCounterReset;              // 0x1040
                unsigned char unused4[0x1060 - 0x1042]; // 0x1042
                CaenD16 Iped;                           // 0x1060
                unsigned char unused5[2];               // 0x1062
                CaenD16 RTestAddress;                   // 0x1064
                unsigned char unused6[2];               // 0x1066
                CaenD16 SWComm;                         // 0x1068
                CaenD16 SlideConstant;                  // 0x106a
                unsigned char unused7[4];               // 0x106c
                CaenD16 AAD;                            // 0x1070
                CaenD16 BAD;                            // 0x1072
                unsigned char unused8[12];              // 0x1074
                ThresholdPair Thresholds[16];           // 0x1080-0x10bf
                char unused9[0x8000 - 0x10c0];          // 0x10c0-0x8000
                CaenD16 ROM[0x4000];

                enum ROMIndex
                        {
                        OUI_MSB =       0,
                        OUI =           0x02a / sizeof( short),
                        OUI_LSB =       0x02e / sizeof( short),
                        Version =       0x032 / sizeof( short),
                        BoardIdMSB =    0x036 / sizeof( short),
                        BoardId =       0x03a / sizeof( short),
                        BoardIdLSB =    0x03e / sizeof( short),
                        MBRev =         0x04e / sizeof( short),
                        OBBRev =        0x052 / sizeof( short),
                        SerialMSB =     0xF02 / sizeof( short),
                        SerialLSB =     0xF06 / sizeof( short)
                        };

                enum OutputBufferWordType
                        {
                        OBT_header = 2<<24,
                        OBT_valid_datum = 0,
                        OBT_end_block = 4<<24,
                        OBT_not_valid_datum = 6<<24,
                        };

                enum OutputBufferBits
                        {
                        OBB_GEO_SHIFT = 1<<27,
                        OBB_GEO = 0x1f<<27,
                        OBT_mask = 7<<24,
                        // Header bits
                        OBB_CRATE_SHIFT = 1<<16,
                        OBB_CRATE = 0xff<<16,

                        OBB_CNT_SHIFT = 8,
                        OBB_CNT = 0x3f<<8,

                        // Datum bits
                        OBB_CHANNEL_SHIFT = 1<<17,
                        OBB_CHANNEL = 0xf<<17,
                        OBB_RG = 1<<16, // 4.5 is wrong?
                        OBB_UN = 1<<13,
                        OBB_OV = 1<<12,
                        OBB_ADC = 0xfff<<0,
                        // EOB Bits
                        OBB_EVENT_COUNTER = 0xffffff<<0
                        };

                enum BitSet1Bits
                        {
                        BS1_BerrFlag = 1<<3,
                        BS1_SelAddr = 1<<4,
                        BS1_SoftReset = 1<<7
                        };

                enum BitSet2Bits
                        {
                        BS2_TestMem = 1<<0,
                        BS2_Offline = 1<<1,
                        BS2_ClearData = 1<<2,
                        BS2_OverRangeEn = 1<<3,

                        BS2_LowThresholdEn = 1<<4,
                        // Not used - 5
                        BS2_TestAc1 = 1<<6,
                        BS2_SlideEn = 1<<7,

                        // Reserved - 8
                        // not used - 9
                        // not used - 10
                        BS2_AutoIncr = 1<<11,

                        BS2_EmptyEnable = 1<<12,
                        BS2_SlideSubEn = 1<<13,
                        BS2_AllTrig = 1<<14
                        // Not used -15
                        };

                enum Status1Bits
                        {
                        ST1_Dready = 1<<0,
                        ST1_GlobalDready = 1<<1,
                        ST1_Busy = 1<<2,
                        ST1_GlobalBusy = 1<<3,
                        ST1_Purged = 1<<5,
                        ST1_EvRdy = 1<<8
                        };

                enum Status2Bits
                        {
                        ST2_BufferEmpty = 1<<1,
                        ST2_BufferFull = 1<<2,
                        // CSEL AND DSEL - get 965A vs 965
                        };
        };

inline drvCaenV965Registers::CaenD16::
operator unsigned short()
        {

        return in_be16( &loc);
        }

inline unsigned short drvCaenV965Registers::CaenD16::
operator =( const unsigned short data)
        {

        out_be16( &loc, data);
        return data;
        }

inline drvCaenV965Registers::CaenD32::
operator unsigned long()
        {

        return in_be32((volatile unsigned int *)&loc);
        }

inline unsigned long drvCaenV965Registers::CaenD32::
operator =( const unsigned long data)
        {

        out_be32((volatile unsigned int *)&loc, data);
        return data;
        }

inline unsigned short drvCaenV965Registers::
getIped()
        {

        return Iped;
        }

inline void drvCaenV965Registers::
setIped(unsigned short val)
        {

        Iped = val;
        }

inline void drvCaenV965Registers::
enableChannel(int chan, bool enable)
        {
        unsigned short threshold;

        threshold = 0xff & Thresholds[chan].high;
        Thresholds[chan].high = threshold | (enable ? 0 : 0x100);
        threshold = 0xff & Thresholds[chan].low;
        Thresholds[chan].low = threshold | (enable ? 0 : 0x100);
        }

inline void drvCaenV965Registers::
enableLoChannel(int chan, bool enable)
        {
        unsigned short threshold;

        threshold = 0xff & Thresholds[chan].low;
        Thresholds[chan].low = threshold | (enable ? 0 : 0x100);
        }

inline void drvCaenV965Registers::
enableHiChannel(int chan, bool enable)
        {
        unsigned short threshold;

        threshold = 0xff & Thresholds[chan].high;
        Thresholds[chan].high = threshold | (enable ? 0 : 0x100);
        }

inline int drvCaenV965Registers::
getModelNumber()
        {
        
        return((( ROM[BoardIdMSB] & 0xFF) << 16) | (( ROM[BoardId] & 0xFF) << 8) | ( ROM[BoardIdLSB] & 0xFF));
        }

inline long drvCaenV965Registers::
getSerialNumber()
        {
        long rv = 0;

        rv = ROM[SerialMSB] & 0xff;
        rv = ( rv << 8) | (ROM[SerialLSB] & 0xff);
        return rv;
        }
