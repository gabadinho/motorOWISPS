TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#==================================================
# build a support library

LIBRARY_IOC += owisps

# xxxRecord.h will be created from xxxRecord.dbd
#DBDINC += xxxRecord
# install owisps.dbd into <top>/dbd
DBD += owisps.dbd

# specify all source files to be compiled and added to the library
#owisps_SRCS += xxx

owisps_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

