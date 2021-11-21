#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "OWISPSMotorDriver.h"



class MockOWISPSAxis: public OWISPSAxis {
public:
    MockOWISPSAxis(OWISPSController *ctrl): OWISPSAxis(ctrl, 0) {
        this->pC_->shuttingDown_ = 1; // Accessing controller's instance variable to prevent polling
        this->axisType = STEPPER_OPENLOOP;
    }

    void log(int reason, const char *format, ...) {}

    MOCK_METHOD(asynStatus, setIntegerParam, (int, int), (override));
    MOCK_METHOD(asynStatus, getIntegerParam, (int, int*), (override));
    MOCK_METHOD(void, setStatusProblem, (asynStatus), (override));
    MOCK_METHOD(asynStatus, callParamCallbacks, (), (override));

    void updateAxisStatus(char owisps_status) {
        OWISPSAxis::updateAxisStatus(owisps_status);
    }
};



/** Creating a shared global dummy controller.
  * Violates isolation principle of unit-testing, but otherwise it core-dumps...
  */
OWISPSController dummy_ctrl("fake_ctrl", "fake_conn", 2, 1, 1);



TEST(updateAxisStatusMock, ReadyAndStopped) {
    MockOWISPSAxis dummy_axis(&dummy_ctrl);
    EXPECT_CALL(dummy_axis, setStatusProblem(asynSuccess));
    EXPECT_CALL(dummy_axis, getIntegerParam(testing::_, testing::_)).Times(3);
    dummy_axis.updateAxisStatus(OWISPS_STATUS_READY);
}

TEST(updateAxisStatusMock, MovingFromStopped) {
    MockOWISPSAxis dummy_axis(&dummy_ctrl);
    EXPECT_CALL(dummy_axis, setStatusProblem(asynSuccess));
    EXPECT_CALL(dummy_axis, getIntegerParam(testing::_, testing::_)).Times(2);
    EXPECT_CALL(dummy_axis, setIntegerParam(testing::_, 1));
    EXPECT_CALL(dummy_axis, setIntegerParam(testing::_, 0));
    dummy_axis.updateAxisStatus(OWISPS_STATUS_HOMING);
}

