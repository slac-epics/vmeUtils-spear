/* VSAMUtils.c - VME Smart Analog Monitor test utilities
 *
 *      Author:         Dayle Kotturi
 *      Date:           03-29-02
 */

#include "VSAM.h"
#include "VSAMUtils.h"
#include "basicIoOps.h"


int VSAM_testMem (const VSAMMEM * pVSAM)
{
    unsigned long          tmpdata;
    unsigned long          val;
    float                  fval;
    volatile uint32_t     *paddr;
    int i;
    VSAMSHORTS s;
    VSAMCHARS c;

    for (i=0,paddr=(volatile uint32_t *)pVSAM->data; i<32; i++,paddr++) {
        fval = (float)in_be32( (volatile void *)paddr );
        printf ("data[%2d]   Addr = %p, Value = %f\n", 
                 i, paddr,fval );
    }

    for (i=0,paddr=(volatile uint32_t *)pVSAM->range; i<8; i++,paddr++) {
        tmpdata = in_be32((volatile void *)paddr);
        c.a = (CHARAMASK & tmpdata); 
        c.b = (CHARBMASK & tmpdata) >> 8; 
        c.c = (CHARCMASK & tmpdata) >> 16; 
        c.d = (CHARDMASK & tmpdata) >> 24; 
        /* range is an unsigned char, but use int to print out value */
        printf ("range[%2d]    Addr = %p, Values = %d, %d, %d, %d\n", 
               i, paddr, c.a, c.b, c.c, c.d);

    }
    for (i=0,paddr=(volatile uint32_t *)pVSAM->ac; i<16; i++,paddr++) {
        tmpdata = in_be32((volatile void *)paddr);
        s.a = (SHORTAMASK & tmpdata); 
        s.b = (SHORTBMASK & tmpdata) >> 16; 
        printf ("ac[%2d]       Addr = %p, Values = %d, %d\n", 
                i, paddr, s.a, s.b);
    }
    printf ("reset        Addr = %p, Value = %lu\n", 
             &pVSAM->reset, pVSAM->reset);

    printf ("mode_control Addr = %p, Value = %lu\n", 
            &pVSAM->mode_control, pVSAM->mode_control);

    printf ("status       Addr = %p, Value = %lu\n", 
            &pVSAM->status, pVSAM->status);

    printf ("pad          Addr = %p, Value = %lu\n", 
             &pVSAM->pad, pVSAM->pad);
   
    printf ("diag_mode    Addr = %p, Value = %lu\n", 
             &pVSAM->diag_mode, pVSAM->diag_mode);

    for (i=0,paddr=(volatile uint32_t *)pVSAM->padding; i<3; i++,paddr++) {
        val = in_be32( (volatile void *)paddr );
        printf ("padding[%1d]   Addr = %p, Value = %lu\n", 
            i, paddr, val);
    }
 return OK;
}

int checkStatus (const VSAMMEM * pVSAM ) {
    if (pVSAM->status & CALIB_SUCCESS)
        printf ("Calibration OK\n");
    else
        printf ("Calibration ERROR\n");

    printf ("status        Addr = %lu, Value = %lu, Mask = %d\n", 
        (long) &pVSAM->status, pVSAM->status, CALIB_SUCCESS);
return OK;
}

int VSAM_resetMode (VSAMMEM * pVSAM) {
    pVSAM->mode_control = 0x0;
    printf ("mode_control  Addr = %lu, Value = %lu\n", 
        (long) pVSAM->mode_control, pVSAM->mode_control);
    printf ("status        Addr = %lu, Value = %lu\n", 
        (long) &pVSAM->status, pVSAM->status);
return OK;
}

int VSAM_setModeMask(VSAMMEM * pVSAM) {
    pVSAM->mode_control = MODE_MASK;
    printf ("mode_control  Addr = %lu, Value = %lu\n", 
        (long) &pVSAM->mode_control, pVSAM->mode_control);
    printf ("status        Addr = %lu, Value = %lu\n", 
        (long) &pVSAM->status, pVSAM->status);
return OK;
} 

int VSAM_setLittleEndian(VSAMMEM * pVSAM) {
    pVSAM->mode_control |= 0x4;
    printf ("mode_control  Addr = %lu, Value = %lu\n", 
        (long) &pVSAM->mode_control, pVSAM->mode_control);
    printf ("status        Addr = %lu, Value = %lu\n", 
        (long) &pVSAM->status, pVSAM->status);
return OK;
}

int VSAM_setBigEndian(VSAMMEM * pVSAM) {
    pVSAM->mode_control &= ~0x4;
    printf ("mode_control  Addr = %lu, Value = %lu\n", 
        (long) &pVSAM->mode_control, pVSAM->mode_control);
    printf ("status        Addr = %lu, Value = %lu\n",
        (long) &pVSAM->status, pVSAM->status);
return OK;
}

int VSAM_setFastScan(VSAMMEM * pVSAM) {
    pVSAM->mode_control |= 0x1;
    printf ("mode_control  Addr = %lu, Value = %lu\n", 
        (long) &pVSAM->mode_control, pVSAM->mode_control);
    printf ("status        Addr = %lu, Value = %lu\n",
        (long) &pVSAM->status, pVSAM->status);
return OK;
}

int VSAM_setNormalScan(VSAMMEM * pVSAM) {
    pVSAM->mode_control &= ~0x1;
    printf ("mode_control  Addr = %lu, Value = %lu\n", 
        (long) &pVSAM->mode_control, pVSAM->mode_control);
    printf ("status        Addr = %lu, Value = %lu\n",
        (long) &pVSAM->status, pVSAM->status);
return OK;
}     

int VSAM_setFirmwareRev(VSAMMEM * pVSAM) {
    pVSAM->mode_control |= 0x2;
    printf ("mode_control  Addr = %lu, Value = %lu\n", 
        (long) &pVSAM->mode_control, pVSAM->mode_control);
    printf ("status        Addr = %lu, Value = %lu\n",
        (long) &pVSAM->status, pVSAM->status);
return OK;
}     

int VSAM_setAnalogChData(VSAMMEM * pVSAM) {
    pVSAM->mode_control &= ~0x2;     
    printf ("mode_control  Addr = %lu, Value = %lu\n", 
        (long) &pVSAM->mode_control, pVSAM->mode_control);
    printf ("status        Addr = %lu, Value = %lu\n",
        (long) &pVSAM->status, pVSAM->status);
return OK;
}     

