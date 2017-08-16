TOP=../

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#USR_CFLAGS +=

#=============================

# xxxRecord.h will be created from xxxRecord.dbd
#DBDINC += xxxRecord

#=============================

LIBRARY_RTEMS = drvSpearTimestamp
INC     = drvSpearTimestamp.h
# <name>.dbd will be created from <name>Include.dbd
DBD		= drvSpearTimestamp.dbd
DBD	       += devSpearTimestamp.dbd

DB		= spearTimestamp.db spearTimestampMas.db spearTimestampSlv.db

drvSpearTimestamp_SRCS += drvSpearTimestamp.c devSpearTimestamp.c

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

#suppress non-strict warnings
CMPLR = ANSI
