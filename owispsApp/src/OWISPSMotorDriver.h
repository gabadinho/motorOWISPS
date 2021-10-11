/*
FILENAME...   OWISPSMotorDriver.h
USAGE...      Motor driver support (model 3, asyn) for the OWIS PS controller series

Jose G.C. Gabadinho
September 2020
*/

#ifndef _OWISPSMOTORDRIVER_H_
#define _OWISPSMOTORDRIVER_H_

#include "asynMotorController.h"
#include "asynMotorAxis.h"



#define MAX_OWISPS_STRING_SIZE 80

#define AXIS_INIT_PARAMNAME "MOTOR_INIT"
#define AXIS_INIT_VALUEINIT "INIT"

#define AXIS_PREM_PARAMNAME "MOTOR_PREM"
#define AXIS_PREM_VALUEINIT "INIT"
#define AXIS_PREM_VALUEON   "MON"

#define AXIS_POST_PARAMNAME "MOTOR_POST"
#define AXIS_POST_VALUEOFF  "MOFF"



#define OWISPS_STATUS_INITIALIZED 'I'
#define OWISPS_STATUS_DISABLED    'O'
#define OWISPS_STATUS_READY       'R'
#define OWISPS_STATUS_POSTRAP     'T'
#define OWISPS_STATUS_POSSCURVE   'S'
#define OWISPS_STATUS_VELOMODE    'V'
#define OWISPS_STATUS_HOMING      'P'
#define OWISPS_STATUS_RELEASW     'F'
#define OWISPS_STATUS_JOYMODE     'J'
#define OWISPS_STATUS_DISABSW     'B'
#define OWISPS_STATUS_DISABSWERR  'A'
#define OWISPS_STATUS_DISCTRLERR  'M'
#define OWISPS_STATUS_DISTIMEERR  'Z'
#define OWISPS_STATUS_INITACTIVE  'H'
#define OWISPS_STATUS_NOTRELEASED 'U'
#define OWISPS_STATUS_DISMOTERR   'E'
#define OWISPS_STATUS_POSTRAPWMS  'W'
#define OWISPS_STATUS_POSSCURVWMS 'X'
#define OWISPS_STATUS_VELOMODEWMS 'Y'
#define OWISPS_STATUS_VELOMODECPC 'C'
#define OWISPS_STATUS_PIEZOWMS    'N'
#define OWISPS_STATUS_UNKNOWN     '?'

#define OWISPS_REF_IDX       0
#define OWISPS_REF_REFSW     1
#define OWISPS_REF_REFSWIDX  2
#define OWISPS_REF_IDX0      3
#define OWISPS_REF_REFSW0    4
#define OWISPS_REF_REFSWIDX0 5
#define OWISPS_REF_MAXMIN0   6
#define OWISPS_REF_MINMAX0   7

#define OWISPS_LOWLIM_STOP  1
#define OWISPS_LOWLIM_DEC   2
#define OWISPS_HIGHLIM_DEC  4
#define OWISPS_HIGHLIM_STOP 8
#define OWISPS_POWSTG_ERROR 16

#define OWISPS_AXISTYPE_CMD "?MOTYPE%d"

#define OWISPS_AXESSTAT_CMD "?ASTAT"
#define OWISPS_LIMSTAT_CMD  "?ESTAT%d"

#define OWISPS_INIT_CMD "INIT%d"
#define OWISPS_MOFF_CMD "MOFF%d"
#define OWISPS_MON_CMD  "MON%d"
#define OWISPS_STOP_CMD "STOP%d"

#define OWISPS_GETCOUNTER_CMD "?CNT%d"
#define OWISPS_SETCOUNTER_CMD "CNT%d=%d"

#define OWISPS_GETPOSVEL_CMD "?PVEL%d"
#define OWISPS_SETPOSVEL_CMD "PVEL%d=%d"

#define OWISPS_ABSCOORD_CMD "ABSOL%d"
#define OWISPS_RELCOORD_CMD "RELAT%d"

#define OWISPS_POSSET_CMD    "PSET%d=%d"
#define OWISPS_GETTARGET_CMD "?PSET%d"

#define OWISPS_POSGO_CMD "PGO%d"

#define OWISPS_HOME_CMD "REF%d=%d"

#define OWISPS_VERSION_CMD "?VERSION"
#define OWISPS_MSG_CMD     "?MSG"



enum owispsAxisType {
    UNKNOWN=-1,
    DC_BRUSH,
    STEPPER_OPENLOOP=2,
    STEPPER_CLOSEDLOOP,
    BLDC
};



class OWISPSAxis: public asynMotorAxis {

public:
    OWISPSAxis(class OWISPSController *pC, int axis);

    // These are the methods we override from the base class
    void report(FILE *fp, int level);

    asynStatus move(double position, int relative, double min_velocity, double max_velocity, double acceleration);
    asynStatus home(double minVelocity, double maxVelocity, double acceleration, int forwards);
    asynStatus stop(double acceleration);
    asynStatus setPosition(double position);

    asynStatus poll(bool *moving);

protected:
    // Specific class methods
    void updateAxisStatus(char owisps_status);

    void setStatusProblem(asynStatus status);

    asynStatus executeInit(void);
    asynStatus executePrem(void);
    asynStatus executePost(void);

private:
    OWISPSController *pC_; // Pointer to the asynMotorController to which this axis belongs
  
    owispsAxisType axisType;
    int homingType;
    char axisStatus;
  
friend class OWISPSController;
};



class OWISPSController: public asynMotorController {

public:
    OWISPSController(const char *portName, const char *asynPortName, int numAxes, double movingPollPeriod, double idlePollPeriod);

    // These are the methods we override from the base class
    void report(FILE *fp, int level);

    OWISPSAxis* getAxis(asynUser *pasynUser);
    OWISPSAxis* getAxis(int axisNo);

    asynStatus writeOctet(asynUser *pasynUser, const char *value, size_t maxChars, size_t *nActual);

    asynStatus poll();

protected:
    int driverInitParam;
    int driverPremParam;
    int driverPostParam;
#define NUM_OWISPS_PARAMS 3

friend class OWISPSAxis;
};

#endif // _OWISPSMOTORDRIVER_H_
