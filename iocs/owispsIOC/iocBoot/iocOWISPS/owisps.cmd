# OWIS PS controller-family support

# Load motor record
dbLoadTemplate("owisps.substitutions")

# Configure asyn serial port
drvAsynSerialPortConfigure("SERUSB0", "/dev/ttyUSB0", 0, 0, 0)

asynSetOption ("SERUSB0", 0, "baud",    "9600")
asynSetOption ("SERUSB0", 0, "bits",    "8")
asynSetOption ("SERUSB0", 0, "parity",  "none")
asynSetOption ("SERUSB0", 0, "stop",    "1")
asynSetOption ("SERUSB0", 0, "clocal",  "Y")  # Y = ignore DTR/DSR
asynSetOption ("SERUSB0", 0, "crtscts", "N")  # N = ignore RTS/CTS

# Load asyn record
dbLoadRecords("$(ASYN)/db/asynRecord.db", "P=OWISPS:, R=ASYN1, PORT=SERUSB0, ADDR=0, OMAX=256, IMAX=256")

# Turn on asyn trace
asynSetTraceMask("SERUSB0", 0, 0x03)
asynSetTraceIOMask("SERUSB0", 0, 0x04)

# FeturaPlusCreateController(portName, asynPort, numAxes, movingPollingRate, idlePollingRate)
OWISPSCreateController("OWISPS35", "SERUSB0", 3, 50, 200)

# Turn off asyn trace
asynSetTraceMask("SERUSB0", 0, 0x01)
asynSetTraceIOMask("SERUSB0", 0, 0x00)
