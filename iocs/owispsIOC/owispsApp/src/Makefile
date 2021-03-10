TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#=============================
# Build the IOC application

PROD_IOC = owisps
# owisps.dbd will be created and installed
DBD += owisps.dbd

# owisps.dbd will be made up from these files:
owisps_DBD += base.dbd

# Include dbd files from all support applications:
#owisps_DBD += xxx.dbd

# Add all the support libraries needed by this IOC
#owisps_LIBS += xxx

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

