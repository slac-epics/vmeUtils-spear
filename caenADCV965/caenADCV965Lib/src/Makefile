TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

LIBRARY_IOC_RTEMS += caenADCV965

#INC += drvV965.h
DBD += caenADCV965.dbd

#HTMLS_DIR = .
#HTMLS += V965DeviceSupport.html

USR_CFLAGS += -pedantic 
USR_CFLAGS += -Wshadow -Wpointer-arith -Wbad-function-cast
USR_CFLAGS += -Wredundant-decls -Wnested-externs -Winline

caenADCV965_SRCS += drvV965.cc
caenADCV965_SRCS += devV965.cc


#===============================================================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE
