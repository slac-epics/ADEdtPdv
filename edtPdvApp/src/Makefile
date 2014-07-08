TOP=../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE

#======== DETECTOR LIBRARY ==============

USR_CXXFLAGS_Linux += -D_LINUX -D_x86
USR_CXXFLAGS_Darwin += -D_OSX -D_x86

LIBRARY_IOC_WIN32 += edtPdv
LIBRARY_IOC_Linux += edtPdv
LIBRARY_IOC_Darwin += edtPdv
#LIB_SRCS += prosilica.cpp

LIB_LIBS += pdv

#DBD += prosilicaSupport.dbd

include $(ADCORE)/ADApp/commonLibraryMakefile

#=============================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

