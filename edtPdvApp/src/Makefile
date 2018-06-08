TOP=../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE

ifneq '$(T_A)' 'linuxRT-x86_64'
PROD_IOC_Linux	+= edtGenCpTool
endif

edtGenCpTool_SRCS	+= GenCpPacket.cpp
edtGenCpTool_SRCS	+= edtGenCpTool.cpp

edtGenCpTool_LIBS	+= pdv
edtGenCpTool_SYS_LIBS	+= crypto z

#======== DETECTOR LIBRARY ==============
USR_CXXFLAGS_Linux += -O0

USR_CXXFLAGS_Linux += -D_LINUX -D_x86
USR_CXXFLAGS_Darwin += -D_OSX -D_x86

LIBRARY_IOC_WIN32 += edtPdv
LIBRARY_IOC_Linux += edtPdv
LIBRARY_IOC_Darwin += edtPdv

LIB_SRCS += asynEdtPdvSerial.cpp
LIB_SRCS += edtPdvCamera.cpp
LIB_SRCS += syncDataAcq.cpp
LIB_SRCS += GenCpPacket.cpp

# TODO: Remove this local copy of an EDT src file for
# future pdv lib versions that provide it
LIB_SRCS += promheader.c

LIB_LIBS += pdv

INCLUDE += edtPdvCamera.h

DBD += edtPdvSupport.dbd

include $(ADCORE)/ADApp/commonLibraryMakefile

#=============================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

