# Makefile at top of site support tree
# Makefile,v 1.3 1999/03/09 20:28:34 anj Exp
# scondam: 15-Jul-2019; Exclude gtr as build breaks in base 3.15.5-1.1
#          Apps should link directly TID's versions of gtr:
#          /afs/slac/g/lcls/epics/R3.15.5-1.1/modules/gtr/
#          Only gtr/drvUniverseDmaSup is included in this build 
#          since ssi has dependency on its header files.
#          In the ffuture this dependency should be removed by linking
#          properly with TID's gtr/drvUniverseDmaSup
TOP = .
include $(TOP)/configure/CONFIG

DIRS += drvSpearTimestamp
#DIRS += gtr
DIRS += gtr/drvUniverseDmaSup
DIRS += iocMonLib
DIRS += caenADCV965
DIRS += CaenADCV965_8
DIRS += ssi
# vmeCardRecord before vsam
DIRS += vmeCardRecord
DIRS += vsam

include $(TOP)/configure/RULES_TOP
