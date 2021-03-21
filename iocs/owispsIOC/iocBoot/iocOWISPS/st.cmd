#!../../bin/linux-x86_64/owisps

< envPaths

cd "${TOP}"

## Register all support components
dbLoadDatabase "dbd/owisps.dbd"
owisps_registerRecordDeviceDriver(pdbbase) 

cd "${TOP}/iocBoot/${IOC}"

## motorUtil (allstop & alldone)
dbLoadRecords("$(MOTOR)/db/motorUtil.db", "P=OWISPS:")

##
< owisps.cmd

iocInit()

## motorUtil (allstop & alldone)
motorUtilInit("OWISPS:")

# Boot complete
