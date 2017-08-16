#==============================================================
#
#  Abs:  vxWorks Script to init VSAM configuration 
#
#  Name: VSAMInit.cmd 
#
#  Rem:  This script is used to look for a block of VME
#        Address of assigned to VSAM modules.
#
#        VSAM_init(long *addr,short num,short first)
#
#        addr  - Base A24 address (VME system bus address)
#        num   - Number of cards installed in crate (0-15)
#        first - First card number
#
#  Side: This script must be executed prior
#        to iocInit().
#
#  Facility:  SLAC
#
#  Auth: 18-Feb-1999, K. Luchini     (LUCHINI)
#  Rev:  dd-mmm-yyyy, First Lastname (USERNAME)
#
#--------------------------------------------------------------
#  Mod:
#       dd-mmm-yyyy, First Lastname (USERNAME):
#         comments
#
#==============================================================
#
#
VSAM_init( 0x400000,1,0 )
