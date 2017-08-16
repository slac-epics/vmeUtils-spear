TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

LIBRARY_IOC_RTEMS += CaenADCV965_8

#INC += drvV965.h
DBD += CaenADCV965_8.dbd

USR_CFLAGS += -pedantic 
USR_CFLAGS += -Wshadow -Wpointer-arith -Wbad-function-cast
USR_CFLAGS += -Wredundant-decls -Wnested-externs -Winline

CaenADCV965_8_SRCS += drvV965.cc
CaenADCV965_8_SRCS += devV965.cc

#===============================================================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE
