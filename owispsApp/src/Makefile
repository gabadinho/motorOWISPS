TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#==================================================
# build a support library

LIBRARY_IOC += owispsMotor

# install owisps.dbd into <top>/dbd
DBD += owispsMotor.dbd

INC += OWISPSMotorDriver.h

# specify all source files to be compiled and added to the library
owispsMotor_SRCS += OWISPSMotorDriver.cpp

owispsMotor_LIBS += motor
owispsMotor_LIBS += asyn

owispsMotor_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

