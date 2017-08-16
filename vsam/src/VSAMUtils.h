/* VSAMUtils.h - Function prototypes and variables needed to use
 *               VME Smart Analog Monitor test utilities
 *   
 *      Author:         Dayle Kotturi
 *      Date:           03-29-02
 */
#ifndef VSAMUTILS_H
#define VSAMUTILS_H

#ifdef __cplusplus
extern "C" {
#endif  /* __cplusplus */

#define CHARAMASK (0x000000ff)
#define CHARBMASK (0x0000ff00)
#define CHARCMASK (0x00ff0000)
#define CHARDMASK (0xff000000)
#define SHORTAMASK (0x0000ffff)
#define SHORTBMASK (0xffff0000)


typedef struct {
    short a;
    short b;
} VSAMSHORTS;

typedef struct {
    char a;
    char b;
    char c;
    char d;
} VSAMCHARS;

int VSAM_testMem (const VSAMMEM * pVSAM);
int VSAM_checkStatus (const VSAMMEM * pVSAM );
int VSAM_resetMode (VSAMMEM * pVSAM);
int VSAM_setModeMask(VSAMMEM * pVSAM);
int VSAM_setLittleEndian(VSAMMEM * pVSAM);
int VSAM_setBigEndian(VSAMMEM * pVSAM);
int VSAM_setFastScan(VSAMMEM * pVSAM);
int VSAM_setNormalScan(VSAMMEM * pVSAM);
int VSAM_setFirmwareRev(VSAMMEM * pVSAM);
int VSAM_setAnalogChData(VSAMMEM * pVSAM);

#ifdef __cplusplus
}
#endif  /* __cplusplus */
#endif  /* VSAMUTILS_H */
