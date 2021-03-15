/*
FILENAME...   OWISPSMotorDriver.h
USAGE...      Motor driver support (model 3, asyn) for the OWIS PS controller series

Jose Gabadinho
September 2020
*/

#ifndef _OWISPSMOTORDRIVER_H_
#define _OWISPSMOTORDRIVER_H_

#include "asynMotorController.h"
#include "asynMotorAxis.h"



#define MAX_OWIS_STRING_SIZE 80

#define OwisInitString    "MOTOR_INIT"
#define OwisInitValueInit "INIT"

#define OwisPremString    "MOTOR_PREM"
#define OwisPremValueInit "INIT"
#define OwisPremValueOn   "MON"

#define OwisPostString   "MOTOR_POST"
#define OwisPostValueOff "MOFF"



#define PS35_STATUS_INITIALIZED 'I'
#define PS35_STATUS_DISABLED    'O'
#define PS35_STATUS_READY       'R'
#define PS35_STATUS_POSTRAP     'T'
#define PS35_STATUS_POSSCURVE   'S'
#define PS35_STATUS_VELOMODE    'V'
#define PS35_STATUS_HOMING      'P'
#define PS35_STATUS_RELEASW     'F'
#define PS35_STATUS_JOYMODE     'J'
#define PS35_STATUS_DISABSW     'B'
#define PS35_STATUS_DISABSWERR  'A'
#define PS35_STATUS_DISCTRLERR  'M'
#define PS35_STATUS_DISTIMEERR  'Z'
#define PS35_STATUS_INITACTIVE  'H'
#define PS35_STATUS_NOTRELEASED 'U'
#define PS35_STATUS_DISMOTERR   'E'
#define PS35_STATUS_POSTRAPWMS  'W'
#define PS35_STATUS_POSSCURVWMS 'X'
#define PS35_STATUS_VELOMODEWMS 'Y'
#define PS35_STATUS_VELOMODECPC 'C'
#define PS35_STATUS_PIEZOWMS    'N'
#define PS35_STATUS_UNKNOWN     '?'

#define PS35_REF_IDX       0
#define PS35_REF_REFSW     1
#define PS35_REF_REFSWIDX  2
#define PS35_REF_IDX0      3
#define PS35_REF_REFSW0    4
#define PS35_REF_REFSWIDX0 5
#define PS35_REF_MAXMIN0   6
#define PS35_REF_MINMAX0   7

#define PS35_LOWLIM_STOP  1
#define PS35_LOWLIM_DEC   2
#define PS35_HIGHLIM_DEC  4
#define PS35_HIGHLIM_STOP 8
#define PS35_POWSTG_ERROR 16

#define PS35_AXISTYPE_CMD "?MOTYPE%d"

#define PS35_AXESSTAT_CMD "?ASTAT"
#define PS35_LIMSTAT_CMD  "?ESTAT%d"

#define PS35_INIT_CMD "INIT%d"
#define PS35_MOFF_CMD "MOFF%d"
#define PS35_MON_CMD  "MON%d"
#define PS35_STOP_CMD "STOP%d"

#define PS35_GETCOUNTER_CMD "?CNT%d"
#define PS35_SETCOUNTER_CMD "CNT%d=%d"

#define PS35_GETPOSVEL_CMD "?PVEL%d"
#define PS35_SETPOSVEL_CMD "PVEL%d=%d"

#define PS35_ABSCOORD_CMD "ABSOL%d"
#define PS35_RELCOORD_CMD "RELAT%d"

#define PS35_POSSET_CMD    "PSET%d=%d"
#define PS35_GETTARGET_CMD "?PSET%d"

#define PS35_POSGO_CMD "PGO%d"

#define PS35_HOME_CMD "REF%d=%d"

#define PS35_VERSION_CMD "?VERSION"
#define PS35_MSG_CMD     "?MSG"



enum owisAxisType {
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

    // Specific class methods
    void updateAxisStatus(char ps35_status);

    void setStatusProblem(asynStatus status);

    asynStatus executeInit(void);
    asynStatus executePrem(void);
    asynStatus executePost(void);

private:
    OWISPSController *pC_; // Pointer to the asynMotorController to which this axis belongs
  
    owisAxisType axisType;
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
    int OwisInit_;
    int OwisPrem_;
    int OwisPost_;
#define NUM_OWIS_PARAMS 3

private:

friend class OWISPSAxis;
};

#endif // _OWISPSMOTORDRIVER_H_
