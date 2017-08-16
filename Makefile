# Makefile at top of site support tree
# Makefile,v 1.3 1999/03/09 20:28:34 anj Exp
TOP = .
include $(TOP)/configure/CONFIG

DIRS += drvSpearTimestamp
DIRS += gtr
DIRS += iocMonLib
DIRS += caenADCV965
DIRS += CaenADCV965_8
DIRS += ssi
# vmeCardRecord before vsam
DIRS += vmeCardRecord
DIRS += vsam

include $(TOP)/configure/RULES_TOP
