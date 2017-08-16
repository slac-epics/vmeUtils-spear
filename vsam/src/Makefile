TOP=..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE

# Install database definition files
DBD += devVSAMCard.dbd
DBD += devVSAM.dbd

# Install includes
INC += VSAM.h
INC += VSAMUtils.h

#USR_CFLAGS += -g -DDEBUG

# Link everything into a library:
LIBRARY_IOC_RTEMS   = vsam
LIBRARY_IOC_vxWorks = vsam

# Source files (for depends target):
LIBSRCS += VSAMUtils.c
LIBSRCS += devAiVSAM.c
LIBSRCS += devBiVSAM.c
LIBSRCS += devBoVSAM.c
LIBSRCS += devCardVSAM.c
LIBSRCS += drvVSAM.c

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE
