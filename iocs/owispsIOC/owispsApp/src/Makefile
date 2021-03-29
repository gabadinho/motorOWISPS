TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

# The following are used for debugging messages.
#!USR_CXXFLAGS += -DDEBUG

#=============================
# Build the IOC application

PROD_IOC = owisps
# owisps.dbd will be created and installed
DBD += owisps.dbd

# owisps.dbd will be made up from these files:
owisps_DBD += base.dbd

# Include dbd files from all support applications:
#ifdef ASYN
owisps_DBD += asyn.dbd
owisps_DBD += drvAsynSerialPort.dbd
owisps_DBD += drvAsynIPPort.dbd
#endif
owisps_DBD += motorSupport.dbd
owisps_DBD += owispsMotor.dbd

# Add all the support libraries needed by this IOC
owisps_LIBS += owispsMotor
owisps_LIBS += motor
#ifdef ASYN
owisps_LIBS += asyn
#endif
#ifdef SNCSEQ
owisps_LIBS += seq pv
#endif

# owisps_registerRecordDeviceDriver.cpp derives from owisps.dbd
owisps_SRCS += owisps_registerRecordDeviceDriver.cpp

# Build the main IOC entry point on workstation OSs.
owisps_SRCS_DEFAULT += owispsMain.cpp
owisps_SRCS_vxWorks += -nil-

# Add support from base/src/vxWorks if needed
#owisps_OBJS_vxWorks += $(EPICS_BASE_BIN)/vxComLibrary

# Finally link to the EPICS Base libraries
owisps_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

