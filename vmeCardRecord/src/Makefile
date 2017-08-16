#
#  Abs:  Makefile to build EPICS
#
#  Name: Makefile
#
#  Facility:  PSI
#
#  Auth: dd-mmm-yyyy, Author's Name   (USERNAME)
#  Rev:  dd-mmm-yyyy, Reviewer's Name (USERNAME)
#
#--------------------------------------------------------------
#  Mod:
#        dd-mmm-yyyy, First Lastname (USERNAME):
#            comments
#
#==============================================================
#
TOP=..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#
DBDINC += vmeCardRecord
DBD    += vmeCardRecord.dbd

#HTMLS_DIR = .
#HTMLS  +=  Recordref-vmeCard.html

LIBRARY_IOC_RTEMS   = vmeCardRecord
LIBRARY_IOC_vxWorks = vmeCardRecord

# Add locally compiled object code
vmeCardRecord_SRCS += vmeCardRecord.c

sSubRecord_LIBS = $(EPICS_BASE_IOC_LIBS)

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE
