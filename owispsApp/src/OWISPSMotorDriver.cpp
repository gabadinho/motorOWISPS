/*
FILENAME...   OWISPSMotorDriver.cpp
USAGE...      Motor driver support (model 3, asyn) for the OWIS PS controller series

Jose G.C. Gabadinho
September 2020
*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include "OWISPSMotorDriver.h"

#include <iocsh.h>
#include <epicsThread.h>

#include <asynOctetSyncIO.h>

#include <epicsExport.h>



static const char *driverName = "OWISPSController";

/** Creates a new OWISPSController object.
  *
  * \param[in] portName          The name of the asyn port that will be created for this driver
  * \param[in] asynPortName      The name of the drvAsynIPPPort that was created previously to connect to the OWIS PS controller 
  * \param[in] numAxes           The number of axes that this controller supports 
  * \param[in] movingPollPeriod  The time between polls when any axis is moving 
  * \param[in] idlePollPeriod    The time between polls when no axis is moving 
  */
OWISPSController::OWISPSController(const char *portName, const char *asynPortName, int numAxes, double movingPollPeriod, double idlePollPeriod)
    :asynMotorController(portName, numAxes, NUM_OWISPS_PARAMS, 
                         asynOctetMask, 
                         asynOctetMask,
                         ASYN_CANBLOCK | ASYN_MULTIDEVICE, 
                         1, /* autoconnect */
                         0, 0) /* Default priority and stack size */ {
    int axis;
    asynStatus status;
    char eos[10];
    int eos_len;
    static const char *functionName = "OWISPSController";

    createParam(AXIS_INIT_PARAMNAME, asynParamOctet, &driverInitParam);
    createParam(AXIS_PREM_PARAMNAME, asynParamOctet, &driverPremParam);
    createParam(AXIS_POST_PARAMNAME, asynParamOctet, &driverPostParam);

    // Connect to PS controller
    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s:%s: Creating OWIS PS controller %s to asyn %s with %d axes\n", driverName, functionName, portName, asynPortName, numAxes);
    status = pasynOctetSyncIO->connect(asynPortName, 0, &pasynUserController_, NULL);
    if (status) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: cannot connect to OWIS PS controller\n", driverName, functionName);
    }

    pasynOctetSyncIO->getInputEos(pasynUserController_, eos, 10, &eos_len);
    if (!eos_len) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s:%s: Setting input acknowledgement to CR\n", driverName, functionName);
        pasynOctetSyncIO->setInputEos(pasynUserController_, "\r", 1);
    }

    pasynOctetSyncIO->getOutputEos(pasynUserController_, eos, 10, &eos_len);
    if (!eos_len) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s:%s: Setting output acknowledgement to CR\n", driverName, functionName);
        pasynOctetSyncIO->setOutputEos(pasynUserController_, "\r", 1);
    }

    // Create the axis objects
    for (axis=0; axis<numAxes; axis++) {
        new OWISPSAxis(this, axis);
    }

    startPoller(movingPollPeriod, idlePollPeriod, 2);
}

/** Reports on status of the driver.
  * If level > 0 then error message, firmware version, axes information is printed.
  *
  * \param[in] fp The file pointer on which report information will be written
  * \param[in] level The level of report detail desired
  */
void OWISPSController::report(FILE *fp, int level) {
    asynStatus status = asynError;

    fprintf(fp, "OWIS PS motor controller %s, numAxes=%d, moving poll period=%f, idle poll period=%f\n", this->portName, numAxes_, movingPollPeriod_, idlePollPeriod_);

    if (level > 0) {
        buildGenericCommand(this->outString_, OWISPS_MSG_CMD);
        status = writeReadController();
        if (status == asynSuccess) {
            fprintf(fp, "    error message=%s\n", this->inString_);
        }

        buildGenericCommand(this->outString_, OWISPS_AXESSTAT_CMD);
        status = writeReadController();
        if (status == asynSuccess) {
            fprintf(fp, "    axes status=%s\n", this->inString_);
        }

        buildGenericCommand(this->outString_, OWISPS_VERSION_CMD);
        status = writeReadController();
        if (status == asynSuccess) {
            fprintf(fp, "    firmware version=%s\n", this->inString_);
        }
    }

    // Call the base class method
    asynMotorController::report(fp, level);
}

/** Returns a pointer to an OWISPSAxis object.
  *
  * \param[in] pasynUser asynUser structure that encodes the axis index number
  *
  * \return OWISPSAxis object or NULL if the axis number encoded in pasynUser is invalid
  */
OWISPSAxis* OWISPSController::getAxis(asynUser *pasynUser) {
    return static_cast<OWISPSAxis*>(asynMotorController::getAxis(pasynUser));
}

/** Returns a pointer to an OWISPSAxis object.
  *
  * \param[in] axisNo Axis index number
  *
  * \return OWISPSAxis object or NULL if the axis number encoded in pasynUser is invalid
  */
OWISPSAxis* OWISPSController::getAxis(int axisNo) {
    return static_cast<OWISPSAxis*>(asynMotorController::getAxis(axisNo));
}

/** Wrapper of writeOctet, to enable the motor at initialization stage (if configured in INIT).
  *
  * \param[in] pasynUser asynUser structure
  * \param[in] value     Value to write
  * \param[in] maxChars  Number of bytes to write
  *
  * \param[out] nActual Number of bytes written
  *
  * \return Result of callParamCallbacks() call
  */
asynStatus OWISPSController::writeOctet(asynUser *pasynUser, const char *value, size_t maxChars, size_t *nActual) {
    int function = pasynUser->reason;
    asynStatus status;
    OWISPSAxis *pAxis = getAxis(pasynUser);
    
    status = asynMotorController::writeOctet(pasynUser, value, maxChars, nActual);
    if ((status == asynSuccess) && (function == driverInitParam)) {
        pAxis->executeInit();
    }

    return callParamCallbacks();
}

/** Polls the controller.
  * Reads the joint axes state and updates them.
  *
  * \return Result of writeReadController() call
  */
asynStatus OWISPSController::poll() {
    asynStatus status;
    OWISPSAxis* axis;

    buildGenericCommand(this->outString_, OWISPS_AXESSTAT_CMD);
    status = writeReadController();
    if (status == asynSuccess) {
        int l = strlen(this->inString_);
        for (int i=0; i<l; i++) {
            axis = getAxis(i);
            if (axis) {
                axis->updateAxisStatus(this->inString_[i]);
            }
        }
    }

    return status;
}

/** The following methods generate a command string to be sent to the controller.
  *
  */
bool OWISPSController::buildGenericCommand(char *buffer, const char *command_format) {
    if ((!buffer) || (!command_format)) {
        return false;
    }
    sprintf(buffer, "%s", command_format);
    return true;
}



// These are the OWISPSAxis methods

/** Creates a new OWISPSAxis object.
  *
  * \param[in] pC Pointer to the OWISPSController to which this axis belongs
  * \param[in] axisNo Index number of this axis, range 0 to pC->numAxes_-1
  */
OWISPSAxis::OWISPSAxis(OWISPSController *pC, int axisNo): asynMotorAxis(pC, axisNo), pC_(pC) {
    asynStatus status;

    this->axisType = UNKNOWN;
    this->axisStatus = OWISPS_STATUS_UNKNOWN;
    this->homingType = OWISPS_REF_REFSW0;

    buildGenericCommand(pC->outString_, OWISPS_AXISTYPE_CMD, axisNo);
    status = pC->writeReadController();
    if (status == asynSuccess) {
        this->axisType = (owispsAxisType)atoi(pC->inString_);

        buildGenericCommand(pC->outString_, OWISPS_LIMSTAT_CMD, axisNo);
        status = pC->writeReadController();
        if (status == asynSuccess) {
            int lim_switches = atoi(pC->inString_);
            if (lim_switches & OWISPS_POWSTG_ERROR) {
                setStatusProblem(asynError);
            }
        } else {
            setStatusProblem(status);
        }
    }

    if (this->axisType == UNKNOWN) {
        setIntegerParam(pC->motorStatusCommsError_, 1);
    }
  
    callParamCallbacks();
}

/** Reports on status of the axis.
  * If level > 0 then detailed axis information (type, homing, status, readback, etc.) is printed.
  *
  * \param[in] fp The file pointer on which report information will be written
  * \param[in] level The level of report detail desired
  */
void OWISPSAxis::report(FILE *fp, int level) {
    asynStatus status = asynError;
    char axis_status=' ';
    int lim_switches=0, readback_counter=0, target=0, velocity=0;

    if (level > 0) {
        OWISPSController::buildGenericCommand(pC_->outString_, OWISPS_AXESSTAT_CMD);
        status = pC_->writeReadController();
        if (status == asynSuccess) {
            if (strlen(pC_->inString_) > (unsigned)this->axisNo_) {
                axis_status = pC_->inString_[this->axisNo_];
            }
        }

        buildGenericCommand(pC_->outString_, OWISPS_LIMSTAT_CMD, this->axisNo_);
        status = pC_->writeReadController();
        if (status == asynSuccess) {
            lim_switches = atoi(pC_->inString_);
        }

        buildGenericCommand(pC_->outString_, OWISPS_GETCOUNTER_CMD, this->axisNo_);
        status = pC_->writeReadController();
        if (status == asynSuccess) {
            readback_counter = atoi(pC_->inString_);
        }

        buildGenericCommand(pC_->outString_, OWISPS_GETTARGET_CMD, this->axisNo_);
        status = pC_->writeReadController();
        if (status == asynSuccess) {
            target = atoi(pC_->inString_);
        }

        buildGenericCommand(pC_->outString_, OWISPS_GETPOSVEL_CMD, this->axisNo_);
        status = pC_->writeReadController();
        if (status == asynSuccess) {
            velocity = atoi(pC_->inString_);
        }

        fprintf(fp,
            "  axis %d\n"
            "    type = %d\n"
            "    homing type = %d\n"
            "    current status = %c\n"
            "    limit switches = %x\n"
            "    readback = %d\n"
            "    target = %d\n"
            "    velocity = %d\n",
            this->axisNo_,
            this->axisType,
            this->homingType,
            axis_status,
            lim_switches,
            readback_counter,
            target,
            velocity);

    } else {
        fprintf(fp,
            "  axis %d\n"
            "    type = %d\n"
            "    homing type = %d\n"
            "    last status = %c\n",
            this->axisNo_,
            this->axisType,
            this->homingType,
            this->axisStatus);
    }

    asynMotorAxis::report(fp, level);
}

/** Moves the axis to a different target position, executing the desired user operation defined in PREM.
  * Warning: only implemented for stepper-motors in open-loop!
  *
  * \param[in] position      The desired target position
  * \param[in] relative      1 for relative position
  * \param[in] minVelocity   Motion parameter
  * \param[in] maxVelocity   Motion parameter
  * \param[in] acceleration  Motion parameter
  *
  * \return Result of callParamCallbacks() call
  */
asynStatus OWISPSAxis::move(double position, int relative, double minVelocity, double maxVelocity, double acceleration) {
    asynStatus status = asynError;
    int is_disabled = (this->axisStatus==OWISPS_STATUS_UNKNOWN) || (this->axisStatus==OWISPS_STATUS_INITIALIZED) || (this->axisStatus==OWISPS_STATUS_DISABLED);

    switch(this->axisType) {
        case STEPPER_OPENLOOP:
            status = executePrem();
            if ((status == asynError) && (is_disabled)) {
                // Motor wasn't ready and no prem command defined
            } else {
                setIntegerParam(pC_->motorStatusDone_, 0);

                if (relative) {
                    buildGenericCommand(pC_->outString_, OWISPS_RELCOORD_CMD, this->axisNo_);
                    status = pC_->writeController();
                } else {
                    buildGenericCommand(pC_->outString_, OWISPS_ABSCOORD_CMD, this->axisNo_);
                    status = pC_->writeController();
                }

                if (status == asynSuccess) {
                    buildMoveCommand(pC_->outString_, this->axisNo_, position);
                    status = pC_->writeController();

                    if (status == asynSuccess) {
                        buildGenericCommand(pC_->outString_, OWISPS_POSGO_CMD, this->axisNo_);
                        status = pC_->writeController();
                    }
                }
            }

            setStatusProblem(status);
            break;

        default:
            setStatusProblem(asynError);
            break;
    }

    return callParamCallbacks();
}

/** Starts the axis homing procedure, executing the desired user operation defined in PREM.
  * Warning: only implemented for stepper-motors in open-loop!
  * Currently hardwired to OWISPS_REF_REFSW0, equivalent to "REF?=4".
  *
  * \param[in] minVelocity   Motion parameter
  * \param[in] maxVelocity   Motion parameter
  * \param[in] acceleration  Motion parameter
  * \param[in] forwards      1 if user wants to home forward, 0 for reverse
  *
  * \return Result of callParamCallbacks() call
  */
asynStatus OWISPSAxis::home(double minVelocity, double maxVelocity, double acceleration, int forwards) {
    asynStatus status = asynError;
    int is_disabled = (this->axisStatus==OWISPS_STATUS_UNKNOWN) || (this->axisStatus==OWISPS_STATUS_INITIALIZED) || (this->axisStatus==OWISPS_STATUS_DISABLED);

    switch(this->axisType) {
        case STEPPER_OPENLOOP:
            status = executePrem();
            if ((status == asynError) && (is_disabled)) {
                // Motor wasn't ready and no prem command defined
            } else {
                setIntegerParam(pC_->motorStatusHome_, 1);
                setIntegerParam(pC_->motorStatusDone_, 0);
                buildHomeCommand(pC_->outString_, this->axisNo_, this->homingType);
                status = pC_->writeController();
            }

            setStatusProblem(status);
            break;

        default:
            setStatusProblem(asynError);
            break;
    }

    return callParamCallbacks();
}

/** Stops an ongoing motion.
  *
  * \param[in] acceleration  Motion parameter
  *
  * \return Result of callParamCallbacks() call
  */
asynStatus OWISPSAxis::stop(double acceleration) {
    asynStatus status = asynError;

    if (this->axisType != UNKNOWN) {
        buildGenericCommand(pC_->outString_, OWISPS_STOP_CMD, this->axisNo_);
        status = pC_->writeController();
    }

    setStatusProblem(status);

    return callParamCallbacks();
}

/** Forces the axis readback position to some value.
  *
  * \param[in] position  The desired readback position
  *
  * \return Result of callParamCallbacks() call
  */
asynStatus OWISPSAxis::setPosition(double position) {
    asynStatus status = asynError;

    buildSetPositionCommand(pC_->outString_, this->axisNo_, position);
    status = pC_->writeController();

    setStatusProblem(status);

    return callParamCallbacks();
}

/** Polls the axis.
  * Reads the limits state and readback position and calls setIntegerParam() or setDoubleParam() for each item that it polls.
  *
  * \param[out] moving A flag that is set indicating that the axis is moving (1) or done (0).
  *
  * \return Result of callParamCallbacks() call
  */
asynStatus OWISPSAxis::poll(bool *moving) { 
    asynStatus status = asynError;
    int at_limit, ismoving, lim_switches, readback_counter;

    if (this->axisType != UNKNOWN) {

        ismoving = ((this->axisStatus == OWISPS_STATUS_POSTRAP)    ||
                    (this->axisStatus == OWISPS_STATUS_POSSCURVE)  ||
                    (this->axisStatus == OWISPS_STATUS_HOMING)     ||
                    (this->axisStatus == OWISPS_STATUS_RELEASW)    ||
                    (this->axisStatus == OWISPS_STATUS_POSTRAPWMS) ||
                    (this->axisStatus == OWISPS_STATUS_POSSCURVWMS)  );
        *moving = ismoving;

        buildGenericCommand(pC_->outString_, OWISPS_LIMSTAT_CMD, this->axisNo_);
        status = pC_->writeReadController();
        if (status == asynSuccess) {
            lim_switches = atoi(pC_->inString_);
            if (lim_switches & OWISPS_POWSTG_ERROR) { // Disconnected or power error?
                status = asynError;

            } else {
                pC_->getIntegerParam(this->axisNo_, pC_->motorStatusLowLimit_, &at_limit);
                if ( (lim_switches & OWISPS_LOWLIM_DEC) && (!at_limit)) {
                    setIntegerParam(pC_->motorStatusLowLimit_, 1);
                } else if ( !(lim_switches & OWISPS_LOWLIM_DEC) && (at_limit)) {
                    setIntegerParam(pC_->motorStatusLowLimit_, 0);
                }
                pC_->getIntegerParam(this->axisNo_, pC_->motorStatusHighLimit_, &at_limit);
                if ( (lim_switches & OWISPS_HIGHLIM_DEC) && (!at_limit)) {
                    setIntegerParam(pC_->motorStatusHighLimit_, 1);
                } else if ( !(lim_switches & OWISPS_HIGHLIM_DEC) && (at_limit)) {
                    setIntegerParam(pC_->motorStatusHighLimit_, 0);
                }

                buildGenericCommand(pC_->outString_, OWISPS_GETCOUNTER_CMD, this->axisNo_);
                status = pC_->writeReadController();
                if (status == asynSuccess) {
                    readback_counter = atoi(pC_->inString_);
                    setDoubleParam(pC_->motorPosition_, readback_counter);
                }
            }
        }
    }

    setStatusProblem(status);

    return callParamCallbacks();
}

/** The following methods generate a command string to be sent to the controller.
  *
  */
bool OWISPSAxis::buildGenericCommand(char *buffer, const char *command_format, int axis) {
    if ((!buffer) || (!command_format)) {
        return false;
    }
    sprintf(buffer, command_format, axis+1);
    return true;
}

bool OWISPSAxis::buildMoveCommand(char *buffer, int axis, double position) {
    if (!buffer) {
        return false;
    }
    sprintf(buffer, OWISPS_POSSET_CMD, axis+1, (int)position);
    return true;

}

bool OWISPSAxis::buildSetPositionCommand(char *buffer, int axis, double position) {
    if (!buffer) {
        return false;
    }
    sprintf(buffer, OWISPS_SETCOUNTER_CMD, axis+1, (int)position);
    return true;

}

bool OWISPSAxis::buildHomeCommand(char *buffer, int axis, int home_type) {
    if (!buffer) {
        return false;
    }

    sprintf(buffer, OWISPS_HOME_CMD, axis+1, home_type);
    return true;
}

/** Updates the axis status. Calls setIntegerParam() for moving, done, home, homed.
  *
  * \param[in] owisps_status Axis status, from controller
  */
void OWISPSAxis::updateAxisStatus(char owisps_status) {
    int status_done, status_home, status_moving;

    if (this->axisType != UNKNOWN) {

        this->axisStatus = owisps_status;

        switch(owisps_status) {
            case OWISPS_STATUS_UNKNOWN:
                setStatusProblem(asynError);
                break;

            case OWISPS_STATUS_READY:
                setStatusProblem(asynSuccess);

                pC_->getIntegerParam(this->axisNo_, pC_->motorStatusMoving_, &status_moving);
                if (status_moving) {
                    this->setIntegerParam(pC_->motorStatusMoving_, 0);
                }

                pC_->getIntegerParam(this->axisNo_, pC_->motorStatusDone_, &status_done);
                if (!status_done) {
                    setIntegerParam(pC_->motorStatusDone_, 1);
                    executePost();
                }

                pC_->getIntegerParam(this->axisNo_, pC_->motorStatusHome_, &status_home);
                if (status_home) {
                    setIntegerParam(pC_->motorStatusHome_, 0);
                    setIntegerParam(pC_->motorStatusHomed_, 1);
                }
                break;

            case OWISPS_STATUS_POSTRAP:
            case OWISPS_STATUS_POSSCURVE:
            case OWISPS_STATUS_HOMING:
            case OWISPS_STATUS_RELEASW:
            case OWISPS_STATUS_POSTRAPWMS:
            case OWISPS_STATUS_POSSCURVWMS:
                setStatusProblem(asynSuccess);

                pC_->getIntegerParam(this->axisNo_, pC_->motorStatusMoving_, &status_moving);
                if (!status_moving) {
                    setIntegerParam(pC_->motorStatusMoving_, 1);
                }
                pC_->getIntegerParam(this->axisNo_, pC_->motorStatusDone_, &status_done);
                if (status_done) {
                    setIntegerParam(pC_->motorStatusDone_, 0);
                }
                break;
            
        }
    }
}

/** Raises the motor record problem status.
  *
  * \param[in] status Last operation status
  */
void OWISPSAxis::setStatusProblem(asynStatus status) {
    int status_problem;

    pC_->getIntegerParam(this->axisNo_, pC_->motorStatus_, &status_problem);
    if ((status != asynSuccess) && (!status_problem)) {
        setIntegerParam(pC_->motorStatusProblem_, 1);
    }
    if ((status == asynSuccess) && (status_problem)) {
        setIntegerParam(pC_->motorStatusProblem_, 0);
    }
}


/** Initializes the axis, if motor record INIT field equals to "INIT".
  *
  * \return Result of either getStringParam() or writeController() calls
  */
asynStatus OWISPSAxis::executeInit(void) {
    asynStatus status = asynError;
    char init[MAX_OWISPS_STRING_SIZE]; // Motor record INIT field

    if (pC_->getStringParam(this->axisNo_, pC_->driverInitParam, (int)sizeof(init), init) == asynSuccess) {
        if (strlen(init)) {
            if (!strcmp(init, AXIS_INIT_VALUEINIT)) {
                buildGenericCommand(pC_->outString_, OWISPS_INIT_CMD, this->axisNo_);
                status = pC_->writeController();
            }

            setStatusProblem(status);
        }
    }

    return status;
}

/** Initializes or enables the axis, if motor record PREM field equals to "INIT" or "MON".
  *
  * \return Result of either getStringParam() or writeController() calls
  */
asynStatus OWISPSAxis::executePrem(void) {
    asynStatus status = asynError;
    char prem[MAX_OWISPS_STRING_SIZE]; // Motor record PREM field

    if (pC_->getStringParam(this->axisNo_, pC_->driverPremParam, (int)sizeof(prem), prem) == asynSuccess) {
        if (strlen(prem)) {
            if (!strcmp(prem, AXIS_PREM_VALUEINIT)) {
                buildGenericCommand(pC_->outString_, OWISPS_INIT_CMD, this->axisNo_);
                status = pC_->writeController();
            } else if (!strcmp(prem, AXIS_PREM_VALUEON)) {
                buildGenericCommand(pC_->outString_, OWISPS_MON_CMD, this->axisNo_);
                status = pC_->writeController();
            }

            setStatusProblem(status);
        }
    }

    return status;
}

/** Disables the axis, if motor record POST field equals to "MOFF".
  *
  * \return Result of either getStringParam() or writeController() calls
  */
asynStatus OWISPSAxis::executePost(void) {
    asynStatus status = asynError;
    char post[MAX_OWISPS_STRING_SIZE]; // Motor record POST field

    if (pC_->getStringParam(this->axisNo_, pC_->driverPostParam, (int)sizeof(post), post) == asynSuccess) {
        if (strlen(post)) {
            if (!strcmp(post, AXIS_POST_VALUEOFF)) {
                buildGenericCommand(pC_->outString_, OWISPS_MOFF_CMD, this->axisNo_);
                status = pC_->writeController();
            }

            setStatusProblem(status);
        }
    }

    return status;
}



/** Creates a new OWISPSController object.
  * Configuration command, called directly or from iocsh
  *
  * \param[in] portName          The name of the asyn port that will be created for this driver
  * \param[in] asynPortName      The name of the drvAsynIPPort/drvAsynSerialPortConfigure that was created previously to connect to the OWIS controller 
  * \param[in] numAxes           The number of axes that this controller supports 
  * \param[in] movingPollPeriod  The time in ms between polls when any axis is moving
  * \param[in] idlePollPeriod    The time in ms between polls when no axis is moving 
  *
  * \return Always asynSuccess
  */
extern "C" int OWISPSCreateController(const char *portName, const char *asynPortName, int numAxes,  int movingPollPeriod, int idlePollPeriod) {
    new OWISPSController(portName, asynPortName, numAxes, movingPollPeriod/1000., idlePollPeriod/1000.);
    return asynSuccess;
}

/** Code for iocsh registration */
static const iocshArg OWISPSCreateControllerArg0 = { "Port name", iocshArgString };
static const iocshArg OWISPSCreateControllerArg1 = { "Asyn port name", iocshArgString };
static const iocshArg OWISPSCreateControllerArg2 = { "Number of axes", iocshArgInt };
static const iocshArg OWISPSCreateControllerArg3 = { "Moving poll period (ms)", iocshArgInt };
static const iocshArg OWISPSCreateControllerArg4 = { "Idle poll period (ms)", iocshArgInt };
static const iocshArg * const OWISPSCreateControllerArgs[] = { &OWISPSCreateControllerArg0,
                                                               &OWISPSCreateControllerArg1,
                                                               &OWISPSCreateControllerArg2,
                                                               &OWISPSCreateControllerArg3,
                                                               &OWISPSCreateControllerArg4 };
static const iocshFuncDef OWISPSCreateControllerDef = { "OWISPSCreateController", 5, OWISPSCreateControllerArgs };
static void OWISPSCreateControllerCallFunc(const iocshArgBuf *args) {
    OWISPSCreateController(args[0].sval, args[1].sval, args[2].ival, args[3].ival, args[4].ival);
}

static void OWISPSControllerRegister(void) {
    iocshRegister(&OWISPSCreateControllerDef, OWISPSCreateControllerCallFunc);
}

extern "C" {
    epicsExportRegistrar(OWISPSControllerRegister);
}

