#==============================================================
#
#  Abs:  vxWorks startup Script to load USER db and records
#
#  Name: db.cmd 
#
#  Side: 
#        The macros used in the template file
#        provides the substitutions for the test
#        application. However, you may wish to 
#        change these to suite your own application.
#        Also, the template file specifies the location
#        of "db" files and you may need to change this
#        path to suit your needs. 
#        
#          S -  station name (ie: target node name) 
#          D -  card number used for configuration (0-15)
#          C -  channel number
#
#  Facility:  SLAC
#
#  Auth: 12-Apr-2000, K. Luchini   (LUCHINI)
#  Rev:  dd-mmm-yyy
#
#--------------------------------------------------------------
#  Mod:
#       dd-mmm-yyyy, First Lastname (USERNAME):
#         comment
#
#==============================================================
#
dbLoadDatabase("../../dbd/vsamApp.dbd")
dbLoadTemplate("/db/ioc_vsam0name.template")

