TOP=..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

# The files in this directory are installed first because they are
# used both by the NDPluginColorConvert and by the EDTpdv driver
#INC += changeme.h

# Note, the following files were manually copied from the /opt/EDTpdv  directory
# to this directory after building
ifeq (linux-x86_64, $(findstring linux-x86_64, $(T_A)))
LIB_INSTALLS_Linux += ../os/linux-x86_64/libpdv.a

else ifeq (linux-x86, $(findstring linux-x86, $(T_A)))
LIB_INSTALLS_Linux += ../os/linux-x86/libpdv.a

endif

#=============================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

