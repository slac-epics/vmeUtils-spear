/* $Id: svgm.c,v 1.5 2009/08/13 18:54:34 saa Exp $ */

/* Synergy VGM specific pieces */

/* Till Straumann <strauman@slac.stanford.edu> */

#ifdef __rtems__
#include <rtems.h>
#include <bsp.h>
#include <bsp/uart.h>
#undef __BSD_VISIBLE
#define __BSD_VISIBLE 1
#include <sys/socket.h>
#include <sys/sockio.h>
#include <net/if.h>
#include <net/if_types.h>
#include <net/if_dl.h>
#include <net/ethernet.h>
#include <fcntl.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>

#include <devLib.h>
#include <devBusMapped.h>
#include <subRecord.h>
#include <registryFunction.h>

#define BUFLEN 2000

#ifdef RTEMS_BSP_PGM_EXEC_AFTER /* only defined on uC5282 */
#define RTEMSREBOOT(x) bsp_reset(0)
#elif   (defined(__PPC__) && ((__RTEMS_MAJOR__ > 4) \
         || (__RTEMS_MAJOR__ == 4 && __RTEMS_MINOR__ > 9) \
         || (__RTEMS_MAJOR__ == 4 && __RTEMS_MINOR__ == 9 && __RTEMS_REVISION__ > 0)))
#define RTEMSREBOOT(x) bsp_reset()
#else
#define RTEMSREBOOT(x) rtemsReboot()
#endif

static int
getEtherAddr(char *ifnam, char *abuf)
{
int  rval = -1;
void *buf = 0;
int  sd   = -1;
struct ifreq  *pifr;
struct ifconf ifconf;
struct sockaddr *sa;
struct sockaddr_dl *sdl;

	if ( !(buf=malloc(BUFLEN)) ) {
		fprintf(stderr,"getEtherAddr: no memory");
		return -1;
	}

	if ( (sd=socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) {
		perror("getEtherAddr -- socket()");
		goto cleanup;
	}

	ifconf.ifc_len = BUFLEN;
	ifconf.ifc_buf = buf;

	if ( ioctl(sd, SIOCGIFCONF, &ifconf) ) {
		perror("getEtherAddr -- ioctl()");
		goto cleanup;
	}

	for ( pifr = buf;
		  (void*) pifr < buf + ifconf.ifc_len ;
		  pifr = sa->sa_len > sizeof(*sa) ? (void *)((caddr_t)sa + sa->sa_len) : pifr + 1 ) {
		sa = &pifr->ifr_addr;
		if ( ifnam && strcmp( pifr->ifr_name, ifnam ) )
			continue; 	/* no match */
		if ( AF_LINK != sa->sa_family )
			continue;   /* not a link level address */
		sdl = (struct sockaddr_dl *)sa;
		if ( IFT_ETHER != sdl->sdl_type || ETHER_ADDR_LEN != sdl->sdl_alen )
			continue;	/* not an ethernet interface */
		/* found it */
		memcpy( abuf, LLADDR(sdl), ETHER_ADDR_LEN );
		rval = 0;
		break;
	}

cleanup:
	if ( sd>= 0 )
		close(sd);
	free(buf);
	return rval;
}

int
svgmSysReset(subRecord *psub)
{
	if ( psub->val ) {
		RTEMSREBOOT();
	}
	return 0;
}
/*
 * The BSP has a feature which allows you to reboot a board
 * with a break condition on the serial line. This is handy, especially
 * on IOCs w/o a CANbus controlled crate since it works
 * on an otherwise pretty dead CPU (as long as interrupts still work).
 *
 * However, the terminal server sends out a break when it's power cycled
 * causing inadvertant CPU reboot.  For production CPUs, this feature
 * must be turned off.
 */
int rebootOnBreak = 0;
void svgmUartBreakCallback(int uartMinor,
                           unsigned uartRBRLSRStatus,
                           void  *termiosPrivatePtr,
                           void  *private)
{
  if (rebootOnBreak) RTEMSREBOOT();
}

int
svgmUartBreak(subRecord *psub)
{
  if (psub->a < 0.5) {
    BSP_UartBreakCbRec cb_iocarg;
    int fd = open("/dev/console",O_RDWR);

    psub->a = 1.0;
    if (!fd) return -1;
    cb_iocarg.handler = svgmUartBreakCallback;
    cb_iocarg.private = 0;
    ioctl(fd, BIOCSETBREAKCB, &cb_iocarg);
    close(fd);
  }
  if ( psub->val > 0.5) rebootOnBreak = 1;
  else                  rebootOnBreak = 0;
  return 0;
}

#endif


int iocMonSvgmBspSupportInit()
{
int rval = -1;

#ifdef __rtems__
char	mac[ETHER_ADDR_LEN];
char	nvr[4];
	if ( 0 == getEtherAddr("es1",mac) &&
		 0 == devReadProbe(4,  (volatile void*)0xffe9e778, &nvr) &&
		 0 == memcmp(mac+3, nvr, 3) ) {
		/* successfully matched ethernet address to serial number */
		errlogPrintf("SVGM board successfully detected; registering base addresses...\n");
		devBusMappedRegister("svgmRTC", (volatile void*)0xffe9fff0);
		devBusMappedRegister("svgmBoard", (volatile void*)0xffeffe00);
		rval = 0;
	}
	registryFunctionAdd("svgmSysReset",     (void(*)())svgmSysReset);
	registryFunctionAdd("svgmUartBreak",    (void(*)())svgmUartBreak);
#endif

	return rval;
}
