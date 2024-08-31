# Makefile
TOP = ..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
TARGET_CFLAGS += $(CFLAGS_$(OS_CLASS)) $(CFLAGS_$(T_A))

############################################
#  Configuration Options
############################################

INC += drvRTEMSDmaSup.h

LIBRARY_IOC_RTEMS_YES += drvRTEMSDmaSup

LIBRARY_RTEMS=$(LIBRARY_IOC_RTEMS_$(HAVE_RTEMS_VMEDMA))

SRCS = drvRTEMSDmaSup.c

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE
