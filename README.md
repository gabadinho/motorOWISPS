# motorOWISPS
EPICS asyn motor support for OWIS PS-family position control unit.

Please refer to the provided example IOC, owispsIOC, for specifics.

### Usage, in short:
1. Create serial asyn port:
	```drvAsynSerialPortConfigure("SERUSB0", "/dev/ttyUSB0", 0, 0, 0)```
2. Configure OWIS PS to above asyn port:
	```OWISPSCreateController("OWISPS35", "SERUSB0", 3, 50, 200)```
3. Load asynMotor DTYP motor record(s):
	```dbLoadTemplate("owisps.substitutions")```

The ```OWISPSCreateController``` command follows the usual API ```(portName, asynPortName, numAxes, movingPollingRate, idlePollingRate)```.

### Extra records:
- ```$(P)$(M)_INIT_CMD```
- ```$(P)$(M)_PREM_CMD```
- ```$(P)$(M)_POST_CMD```

Use the above records to specify the INIT, PREM and POST motion commants:
- ```INIT```, ```MON``` commands for INIT, PREM records.
- ```MOFF``` command for POST records.

### Limitations:
- Only stepper-motors without encoders have been implemented and tested
- Homing is currently hardwired to OWIS method 4
- Changing velocity is not (yet) implemented

