#include <gtest/gtest.h>

#include "OWISPSMotorDriver.h"



#define STRING_BUFFER_SIZE 256



TEST(CommandBuild, AxisMove) {
    char buffer[STRING_BUFFER_SIZE];
    bool res = OWISPSAxis::buildMoveCommand(buffer, 0, 1250);
    ASSERT_STREQ("PSET1=1250", buffer);
    ASSERT_EQ(true, res);
}

TEST(CommandBuild, AxisMoveStart) {
    char buffer[STRING_BUFFER_SIZE];
    bool res = OWISPSAxis::buildGenericCommand(buffer, OWISPS_POSGO_CMD, 1);
    ASSERT_STREQ("PGO2", buffer);
    ASSERT_EQ(true, res);
}

TEST(CommandBuild, AxisMoveAbsoluteMode) {
    char buffer[STRING_BUFFER_SIZE];
    bool res = OWISPSAxis::buildGenericCommand(buffer, OWISPS_ABSCOORD_CMD, 0);
    ASSERT_STREQ("ABSOL1", buffer);
    ASSERT_EQ(true, res);
}

TEST(CommandBuild, AxisMoveRelativeMode) {
    char buffer[STRING_BUFFER_SIZE];
    bool res = OWISPSAxis::buildGenericCommand(buffer, OWISPS_RELCOORD_CMD, 1);
    ASSERT_STREQ("RELAT2", buffer);
    ASSERT_EQ(true, res);
}

TEST(CommandBuild, AxisLimitSwitches) {
    char buffer[STRING_BUFFER_SIZE];
    bool res = OWISPSAxis::buildGenericCommand(buffer, OWISPS_LIMSTAT_CMD, 0);
    ASSERT_STREQ("?ESTAT1", buffer);
    ASSERT_EQ(true, res);
}

TEST(CommandBuild, AxisReadback) {
    char buffer[STRING_BUFFER_SIZE];
    bool res = OWISPSAxis::buildGenericCommand(buffer, OWISPS_GETCOUNTER_CMD, 1);
    ASSERT_STREQ("?CNT2", buffer);
    ASSERT_EQ(true, res);
}

TEST(CommandBuild, AxisSetPosition) {
    char buffer[STRING_BUFFER_SIZE];
    bool res = OWISPSAxis::buildSetPositionCommand(buffer, 1, -6500);
    ASSERT_STREQ("CNT2=-6500", buffer);
    ASSERT_EQ(true, res);
}

TEST(CommandBuild, AxisHome1) {
    char buffer[STRING_BUFFER_SIZE];
    bool res = OWISPSAxis::buildHomeCommand(buffer, 0, OWISPS_REF_REFSW0);
    ASSERT_STREQ("REF1=4", buffer);
    ASSERT_EQ(true, res);
}

TEST(CommandBuild, AxisHome2) {
    char buffer[STRING_BUFFER_SIZE];
    bool res = OWISPSAxis::buildHomeCommand(buffer, 1, OWISPS_REF_IDX);
    ASSERT_STREQ("REF2=0", buffer);
    ASSERT_EQ(true, res);
}

TEST(CommandBuild, AxisStop) {
    char buffer[STRING_BUFFER_SIZE];
    bool res = OWISPSAxis::buildGenericCommand(buffer, OWISPS_STOP_CMD, 1);
    ASSERT_STREQ("STOP2", buffer);
    ASSERT_EQ(true, res);
}

