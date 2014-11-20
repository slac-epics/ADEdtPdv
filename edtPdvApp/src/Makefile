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

#LIB_SRCS += drvAsynEdtPdvSerial.cpp
LIB_SRCS += asynEdtPdvSerial.cpp
LIB_SRCS += edtPdvCamera.cpp
#LIB_SRCS += edtSync.cpp
LIB_SRCS += syncDataAcq.cpp
# TODO: Remove this local copy of an EDT src file for
# future pdv lib versions that provide it
LIB_SRCS += promheader.c

# TODO: Move to new module
#LIB_SRCS += devTimeStampSourceFifo.c
#LIB_SRCS += registerTimeStampSourceFifo.c
#LIB_SRCS += timeStampSourceFifo.c
#DBD += timeStampSourceFifo.dbd

LIB_LIBS += pdv

#INCLUDE += drvAsynEdtPdvSerial.h
INCLUDE += edtPdvCamera.h

# DBD += drvAsynEdtPdvSerial.dbd
DBD += edtPdvSupport.dbd

include $(ADCORE)/ADApp/commonLibraryMakefile

#=============================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

