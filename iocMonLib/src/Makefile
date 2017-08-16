TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#USR_CFLAGS += 

#=============================

# xxxRecord.h will be created from xxxRecord.dbd
#DBDINC += xxxRecord

# <name>.dbd will be created from <name>Include.dbd
DBD += iocMon.dbd

#=============================

LIBRARY = iocMon

# <name>_registerRecordDeviceDriver.cpp will be created from <name>.dbd
#iocMon_SRCS_DEFAULT += -nil-
iocMon_SRCS += iocMonInit.c
iocMon_SRCS_RTEMS += ppc-cpu-temp.c
ifeq ($(T_A),RTEMS-svgm)
iocMon_SRCS_RTEMS += svgm.c
endif

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

