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

LIBRARY_RTEMS += drvRTEMSDmaSup

SRCS_RTEMS += drvRTEMSDmaSup.c
SRCS += $(SRCS_$(T_A))


include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE
