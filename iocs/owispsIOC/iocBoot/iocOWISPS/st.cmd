#!../../bin/linux-x86_64/owisps

#- You may have to change owisps to something else
#- everywhere it appears in this file

#< envPaths

## Register all support components
dbLoadDatabase("../../dbd/owisps.dbd",0,0)
owisps_registerRecordDeviceDriver(pdbbase) 

## Load record instances
dbLoadRecords("../../db/owisps.db","user=gabadinho")

iocInit()

## Start any sequence programs
#seq sncowisps,"user=gabadinho"
