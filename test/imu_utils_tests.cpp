/******************************************************************************
 * Filename:     imu_utils_tests.cpp
 *
 * Author:       Tran Sgt Brandon, Gomez LCpl Greg
 * Organization: Marine Corps Software Factory
 * Created On:   5/21/2026 2:45 PM
 * Description:  This test files validates (1) DegreesToRadians, (2) InertialToGlobal_X and (3) InertialToGlobal_Y functionality.
 *
 ******************************************************************************/

#include "utils.hpp"
#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <cmath>
#include <stdexcept>

using namespace IMUUtils;

TEST(IMUUtils, Convert_Degrees_To_Radians)
{

    struct Case
    {
        double degrees_input;
        double degrees_expected;
    };

    const std::vector<Case> cases = {
        // These tests are pass correct
        {-45.0, M_PI * 7 / 4},
        {0.0, M_PI * 0},
        {45.0, M_PI / 4},
        {90.0, M_PI / 2},
        {135.0, M_PI * 3 / 4},
        {180.0, M_PI},
        {225.0, M_PI * 5 / 4},
        {270.0, M_PI * 3 / 2},
        {315.0, M_PI * 7 / 4},
        {360.0, 0.0},
        {405.0, M_PI / 4}};

    for (const auto &c : cases)
    {
        EXPECT_NEAR(IMUUtils::DegreesToRadians(c.degrees_input), c.degrees_expected, 1e-9);
    };
};
// -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -
TEST(IMUUtils, Convert_Local_XY_Acceleration_To_Global_X_Acceleration)
{
    struct Case
    {
        double theta_input;
        double x_accel_input;
        double y_accel_input;
        double x_accel_output;
    };

    const std::vector<Case> cases = {
        // Translations about 0.0
        {IMUUtils::DegreesToRadians(0.0), 0.0, 0.0, 0.0},
        {IMUUtils::DegreesToRadians(0.0), 0.0, 1.0, 0.0},
        {IMUUtils::DegreesToRadians(0.0), 1.0, 0.0, 1.0},
        {IMUUtils::DegreesToRadians(0.0), 0.0, -1.0, 0.0},
        {IMUUtils::DegreesToRadians(0.0), -1.0, 0.0, -1.0},
        {IMUUtils::DegreesToRadians(0.0), 1.0, 1.0, 1.0},
        {IMUUtils::DegreesToRadians(0.0), -1.0, -1.0, -1.0},
        {IMUUtils::DegreesToRadians(0.0), 1.0, -1.0, 1.0},
        {IMUUtils::DegreesToRadians(0.0), -1.0, 1.0, -1.0},
        // Translations about 45.0
        {IMUUtils::DegreesToRadians(45.0), 0.0, 0.0, 0.0},
        {IMUUtils::DegreesToRadians(45.0), 0.0, 1.0, 1.0 / std::sqrt(2.0)},
        {IMUUtils::DegreesToRadians(45.0), 1.0, 0.0, 1.0 / std::sqrt(2.0)},
        {IMUUtils::DegreesToRadians(45.0), 0.0, -1.0, -1.0 / std::sqrt(2.0)},
        {IMUUtils::DegreesToRadians(45.0), -1.0, 0.0, -1.0 / std::sqrt(2.0)},
        {IMUUtils::DegreesToRadians(45.0), 1.0, 1.0, std::sqrt(2.0)},
        {IMUUtils::DegreesToRadians(45.0), -1.0, -1.0, -std::sqrt(2.0)},
        {IMUUtils::DegreesToRadians(45.0), 1.0, -1.0, 0.0},
        {IMUUtils::DegreesToRadians(45.0), -1.0, 1.0, 0.0},
        // Translations about 90.0
        {IMUUtils::DegreesToRadians(90.0), 0.0, 0.0, 0.0},
        {IMUUtils::DegreesToRadians(90.0), 0.0, 1.0, 1.0},
        {IMUUtils::DegreesToRadians(90.0), 1.0, 0.0, 0.0},
        {IMUUtils::DegreesToRadians(90.0), 0.0, -1.0, -1.0},
        {IMUUtils::DegreesToRadians(90.0), -1.0, 0.0, 0.0},
        {IMUUtils::DegreesToRadians(90.0), 1.0, 1.0, 1.0},
        {IMUUtils::DegreesToRadians(90.0), -1.0, -1.0, -1.0},
        {IMUUtils::DegreesToRadians(90.0), 1.0, -1.0, -1.0},
        {IMUUtils::DegreesToRadians(90.0), -1.0, 1.0, 1.0},
        // Translations about 135.0
        {IMUUtils::DegreesToRadians(135.0), 0.0, 0.0, 0.0},
        {IMUUtils::DegreesToRadians(135.0), 0.0, 1.0, 1 / std::sqrt(2.0)},
        {IMUUtils::DegreesToRadians(135.0), 1.0, 0.0, -1 / std::sqrt(2.0)},
        {IMUUtils::DegreesToRadians(135.0), 0.0, -1.0, -1 / std::sqrt(2.0)},
        {IMUUtils::DegreesToRadians(135.0), -1.0, 0.0, 1 / std::sqrt(2.0)},
        {IMUUtils::DegreesToRadians(135.0), 1.0, 1.0, 0.0},
        {IMUUtils::DegreesToRadians(135.0), -1.0, -1.0, 0.0},
        {IMUUtils::DegreesToRadians(135.0), 1.0, -1.0, -std::sqrt(2.0)},
        {IMUUtils::DegreesToRadians(135.0), -1.0, 1.0, std::sqrt(2.0)},
        // Translations about 180.0
        {IMUUtils::DegreesToRadians(180.0), 0.0, 0.0, 0.0},
        {IMUUtils::DegreesToRadians(180.0), 0.0, 1.0, 0.0},
        {IMUUtils::DegreesToRadians(180.0), 1.0, 0.0, -1.0},
        {IMUUtils::DegreesToRadians(180.0), 0.0, -1.0, 0.0},
        {IMUUtils::DegreesToRadians(180.0), -1.0, 0.0, 1.0},
        {IMUUtils::DegreesToRadians(180.0), 1.0, 1.0, -1.0},
        {IMUUtils::DegreesToRadians(180.0), -1.0, -1.0, 1.0},
        {IMUUtils::DegreesToRadians(180.0), 1.0, -1.0, -1.0},
        {IMUUtils::DegreesToRadians(180.0), -1.0, 1.0, 1.0},
        // Translations about 225.0
        {IMUUtils::DegreesToRadians(225.0), 0.0, 0.0, 0.0},
        {IMUUtils::DegreesToRadians(225.0), 0.0, 1.0, -1.0 / std::sqrt(2.0)},
        {IMUUtils::DegreesToRadians(225.0), 1.0, 0.0, -1.0 / std::sqrt(2.0)},
        {IMUUtils::DegreesToRadians(225.0), 0.0, -1.0, 1.0 / std::sqrt(2.0)},
        {IMUUtils::DegreesToRadians(225.0), -1.0, 0.0, 1.0 / std::sqrt(2.0)},
        {IMUUtils::DegreesToRadians(225.0), 1.0, 1.0, -std::sqrt(2.0)},
        {IMUUtils::DegreesToRadians(225.0), -1.0, -1.0, std::sqrt(2.0)},
        {IMUUtils::DegreesToRadians(225.0), 1.0, -1.0, 0.0},
        {IMUUtils::DegreesToRadians(225.0), -1.0, 1.0, 0.0},
        // Translations about 270.0
        {IMUUtils::DegreesToRadians(270.0), 0.0, 0.0, 0.0},
        {IMUUtils::DegreesToRadians(270.0), 0.0, 1.0, -1.0},
        {IMUUtils::DegreesToRadians(270.0), 1.0, 0.0, 0.0},
        {IMUUtils::DegreesToRadians(270.0), 0.0, -1.0, 1.0},
        {IMUUtils::DegreesToRadians(270.0), -1.0, 0.0, 0.0},
        {IMUUtils::DegreesToRadians(270.0), 1.0, 1.0, -1.0},
        {IMUUtils::DegreesToRadians(270.0), -1.0, -1.0, 1.0},
        {IMUUtils::DegreesToRadians(270.0), 1.0, -1.0, 1.0},
        {IMUUtils::DegreesToRadians(270.0), -1.0, 1.0, -1.0},
        // Translations about 315.0
        {IMUUtils::DegreesToRadians(315.0), 0.0, 0.0, 0.0},
        {IMUUtils::DegreesToRadians(315.0), 0.0, 1.0, -1.0 / std::sqrt(2.0)},
        {IMUUtils::DegreesToRadians(315.0), 1.0, 0.0, 1.0 / std::sqrt(2.0)},
        {IMUUtils::DegreesToRadians(315.0), 0.0, -1.0, 1.0 / std::sqrt(2.0)},
        {IMUUtils::DegreesToRadians(315.0), -1.0, 0.0, -1 / std::sqrt(2.0)},
        {IMUUtils::DegreesToRadians(315.0), 1.0, 1.0, 0.0},
        {IMUUtils::DegreesToRadians(315.0), -1.0, -1.0, 0.0},
        {IMUUtils::DegreesToRadians(315.0), 1.0, -1.0, std::sqrt(2.0)},
        {IMUUtils::DegreesToRadians(315.0), -1.0, 1.0, -std::sqrt(2.0)},

        // Translations beyond 0-360 bounds
        {IMUUtils::DegreesToRadians(405.0), 1.0, 1.0, std::sqrt(2.0)}};

    for (const auto &c : cases)
    {
        EXPECT_NEAR(InertialToGlobal_X(c.theta_input, c.x_accel_input, c.y_accel_input), c.x_accel_output, 1e-9);
    };
};
// ----------------------------------------------------------------------------
TEST(IMUUtils, Convert_Local_xy_Acceleration_To_Global_Y_Acceleration)
{
    struct Case
    {
        double theta_input;
        double x_accel_input;
        double y_accel_input;
        double y_accel_output;
    };

    const std::vector<Case> cases = {
        // Translations about 0.0
        {IMUUtils::DegreesToRadians(0.0), 0.0, 0.0, 0.0},
        {IMUUtils::DegreesToRadians(0.0), 0.0, 1.0, 1.0},
        {IMUUtils::DegreesToRadians(0.0), 1.0, 0.0, 0.0},
        {IMUUtils::DegreesToRadians(0.0), 0.0, -1.0, -1.0},
        {IMUUtils::DegreesToRadians(0.0), -1.0, 0.0, 0.0},
        {IMUUtils::DegreesToRadians(0.0), 1.0, 1.0, 1.0},
        {IMUUtils::DegreesToRadians(0.0), -1.0, -1.0, -1.0},
        {IMUUtils::DegreesToRadians(0.0), 1.0, -1.0, -1.0},
        {IMUUtils::DegreesToRadians(0.0), -1.0, 1.0, 1.0},
        // Translations about 45.0
        {IMUUtils::DegreesToRadians(45.0), 0.0, 0.0, 0.0},
        {IMUUtils::DegreesToRadians(45.0), 0.0, 1.0, 1.0 / std::sqrt(2.0)},
        {IMUUtils::DegreesToRadians(45.0), 1.0, 0.0, -1.0 / std::sqrt(2.0)},
        {IMUUtils::DegreesToRadians(45.0), 0.0, -1.0, -1.0 / std::sqrt(2.0)},
        {IMUUtils::DegreesToRadians(45.0), -1.0, 0.0, 1.0 / std::sqrt(2.0)},
        {IMUUtils::DegreesToRadians(45.0), 1.0, 1.0, 0.0},
        {IMUUtils::DegreesToRadians(45.0), -1.0, -1.0, 0.0},
        {IMUUtils::DegreesToRadians(45.0), 1.0, -1.0, -std::sqrt(2.0)},
        {IMUUtils::DegreesToRadians(45.0), -1.0, 1.0, std::sqrt(2.0)},
        // Translations about 90.0
        {IMUUtils::DegreesToRadians(90.0), 0.0, 0.0, 0.0},
        {IMUUtils::DegreesToRadians(90.0), 0.0, 1.0, 0.0},
        {IMUUtils::DegreesToRadians(90.0), 1.0, 0.0, -1.0},
        {IMUUtils::DegreesToRadians(90.0), 0.0, -1.0, 0.0},
        {IMUUtils::DegreesToRadians(90.0), -1.0, 0.0, 1.0},
        {IMUUtils::DegreesToRadians(90.0), 1.0, 1.0, -1.0},
        {IMUUtils::DegreesToRadians(90.0), -1.0, -1.0, 1.0},
        {IMUUtils::DegreesToRadians(90.0), 1.0, -1.0, -1.0},
        {IMUUtils::DegreesToRadians(90.0), -1.0, 1.0, 1.0},
        // Translations about 135.0
        {IMUUtils::DegreesToRadians(135.0), 0.0, 0.0, 0.0},
        {IMUUtils::DegreesToRadians(135.0), 0.0, 1.0, -1.0 / std::sqrt(2.0)},
        {IMUUtils::DegreesToRadians(135.0), 1.0, 0.0, -1.0 / std::sqrt(2.0)},
        {IMUUtils::DegreesToRadians(135.0), 0.0, -1.0, 1.0 / std::sqrt(2.0)},
        {IMUUtils::DegreesToRadians(135.0), -1.0, 0.0, 1.0 / std::sqrt(2.0)},
        {IMUUtils::DegreesToRadians(135.0), 1.0, 1.0, -std::sqrt(2.0)},
        {IMUUtils::DegreesToRadians(135.0), -1.0, -1.0, std::sqrt(2.0)},
        {IMUUtils::DegreesToRadians(135.0), 1.0, -1.0, 0.0},
        {IMUUtils::DegreesToRadians(135.0), -1.0, 1.0, 0.0},
        // Translations about 180.0
        {IMUUtils::DegreesToRadians(180.0), 0.0, 0.0, 0.0},
        {IMUUtils::DegreesToRadians(180.0), 0.0, 1.0, -1.0},
        {IMUUtils::DegreesToRadians(180.0), 1.0, 0.0, 0.0},
        {IMUUtils::DegreesToRadians(180.0), 0.0, -1.0, 1.0},
        {IMUUtils::DegreesToRadians(180.0), -1.0, 0.0, 0.0},
        {IMUUtils::DegreesToRadians(180.0), 1.0, 1.0, -1.0},
        {IMUUtils::DegreesToRadians(180.0), -1.0, -1.0, 1.0},
        {IMUUtils::DegreesToRadians(180.0), 1.0, -1.0, 1.0},
        {IMUUtils::DegreesToRadians(180.0), -1.0, 1.0, -1.0},
        // Translations about 225.0
        {IMUUtils::DegreesToRadians(225.0), 0.0, 0.0, 0.0},
        {IMUUtils::DegreesToRadians(225.0), 0.0, 1.0, -1.0 / std::sqrt(2.0)},
        {IMUUtils::DegreesToRadians(225.0), 1.0, 0.0, 1.0 / std::sqrt(2.0)},
        {IMUUtils::DegreesToRadians(225.0), 0.0, -1.0, 1.0 / std::sqrt(2.0)},
        {IMUUtils::DegreesToRadians(225.0), -1.0, 0.0, -1.0 / std::sqrt(2.0)},
        {IMUUtils::DegreesToRadians(225.0), 1.0, 1.0, 0.0},
        {IMUUtils::DegreesToRadians(225.0), -1.0, -1.0, 0.0},
        {IMUUtils::DegreesToRadians(225.0), 1.0, -1.0, std::sqrt(2.0)},
        {IMUUtils::DegreesToRadians(225.0), -1.0, 1.0, -std::sqrt(2.0)},
        // Translations about 270.0
        {IMUUtils::DegreesToRadians(270.0), 0.0, 0.0, 0.0},
        {IMUUtils::DegreesToRadians(270.0), 0.0, 1.0, 0.0},
        {IMUUtils::DegreesToRadians(270.0), 1.0, 0.0, 1.0},
        {IMUUtils::DegreesToRadians(270.0), 0.0, -1.0, 0.0},
        {IMUUtils::DegreesToRadians(270.0), -1.0, 0.0, -1.0},
        {IMUUtils::DegreesToRadians(270.0), 1.0, 1.0, 1.0},
        {IMUUtils::DegreesToRadians(270.0), -1.0, -1.0, -1.0},
        {IMUUtils::DegreesToRadians(270.0), 1.0, -1.0, 1.0},
        {IMUUtils::DegreesToRadians(270.0), -1.0, 1.0, -1.0},
        // Translations about 315.0
        {IMUUtils::DegreesToRadians(315.0), 0.0, 0.0, 0.0},
        {IMUUtils::DegreesToRadians(315.0), 0.0, 1.0, 1.0 / std::sqrt(2.0)},
        {IMUUtils::DegreesToRadians(315.0), 1.0, 0.0, 1.0 / std::sqrt(2.0)},
        {IMUUtils::DegreesToRadians(315.0), 0.0, -1.0, -1.0 / std::sqrt(2.0)},
        {IMUUtils::DegreesToRadians(315.0), -1.0, 0.0, -1.0 / std::sqrt(2.0)},
        {IMUUtils::DegreesToRadians(315.0), 1.0, 1.0, std::sqrt(2.0)},
        {IMUUtils::DegreesToRadians(315.0), -1.0, -1.0, -std::sqrt(2.0)},
        {IMUUtils::DegreesToRadians(315.0), 1.0, -1.0, 0.0},
        {IMUUtils::DegreesToRadians(315.0), -1.0, 1.0, 0.0},

        // Translations beyond 0-360 bounds
        {IMUUtils::DegreesToRadians(405.0), 1.0, 1.0, 0.0}}; // cases

    for (const auto &c : cases)
    {
        EXPECT_NEAR(InertialToGlobal_Y(c.theta_input, c.x_accel_input, c.y_accel_input), c.y_accel_output, 1e-9);
    };
};

TEST(IMUUtils, Longitudinal_MetersPerSecondSquared_DegreesPerSecondSquared) {
    double equator = 0.0;
    double austin = 30.2739;
    double caracas = 10.4716;
    double sydney = -33.855782;
    double newZealand = -47.223303;

    double linearEastAccel = 10.0;
    double linearWestAccel = -10.0;

    EXPECT_THROW(IMUUtils::Convert_Global_X_to_DegPerS2(-90.125, 0.0), std::out_of_range);
    EXPECT_THROW(IMUUtils::Convert_Global_X_to_DegPerS2(90.125, 0.0), std::out_of_range);

    EXPECT_NEAR(IMUUtils::Convert_Global_X_to_DegPerS2(equator, linearEastAccel), .000090000009, 1e-9);
    EXPECT_NEAR(IMUUtils::Convert_Global_X_to_DegPerS2(austin, linearEastAccel), .0000777262837788, 1e-9);
    EXPECT_NEAR(IMUUtils::Convert_Global_X_to_DegPerS2(caracas, linearEastAccel), .0000885010692985, 1e-9);
    EXPECT_NEAR(IMUUtils::Convert_Global_X_to_DegPerS2(sydney, linearEastAccel), .0000747398304481, 1e-9);
    EXPECT_NEAR(IMUUtils::Convert_Global_X_to_DegPerS2(newZealand, linearEastAccel), .0000611228607953, 1e-9);

    EXPECT_NEAR(IMUUtils::Convert_Global_X_to_DegPerS2(equator, linearWestAccel), -.000090000009, 1e-9);
    EXPECT_NEAR(IMUUtils::Convert_Global_X_to_DegPerS2(austin, linearWestAccel), -.0000777262837788, 1e-9);
    EXPECT_NEAR(IMUUtils::Convert_Global_X_to_DegPerS2(caracas, linearWestAccel), -.0000885010692985, 1e-9);
    EXPECT_NEAR(IMUUtils::Convert_Global_X_to_DegPerS2(sydney, linearWestAccel), -.0000747398304481, 1e-9);
    EXPECT_NEAR(IMUUtils::Convert_Global_X_to_DegPerS2(newZealand, linearWestAccel), -.0000611228607953, 1e-9);
}

TEST(IMUUtils, Latitudinal_MetersPerSecondSquared_DegreesPerSecondSquared) {

    EXPECT_NEAR(IMUUtils::Convert_Global_Y_to_DegPerS2(10.0), .000090000009, 1e-9);
    EXPECT_NEAR(IMUUtils::Convert_Global_Y_to_DegPerS2(5.0), .0000450000045, 1e-9);
    EXPECT_NEAR(IMUUtils::Convert_Global_Y_to_DegPerS2(1.0), .0000090000009, 1e-9);
    EXPECT_NEAR(IMUUtils::Convert_Global_Y_to_DegPerS2(2.5), .00002250000225, 1e-9);

    EXPECT_NEAR(IMUUtils::Convert_Global_Y_to_DegPerS2(-10.0), -.000090000009, 1e-9);
    EXPECT_NEAR(IMUUtils::Convert_Global_Y_to_DegPerS2(-5.0), -.0000450000045, 1e-9);
    EXPECT_NEAR(IMUUtils::Convert_Global_Y_to_DegPerS2(-1.0), -.0000090000009, 1e-9);
    EXPECT_NEAR(IMUUtils::Convert_Global_Y_to_DegPerS2(-2.5), -.00002250000225, 1e-9);
}

TEST(IMUUtils, KineticUpdates) {
    const auto initialTimePoint = std::chrono::steady_clock::time_point{};
    const auto initialState = IMUUtils::KineticState(initialTimePoint, 0.0, 0.0, 0.0, 0.0);

    const auto secondTimePoint = initialTimePoint + std::chrono::milliseconds(1);
    IMUUtils::KineticState secondState = IMUUtils::Caclulate_Kinetic_Update(initialState, 1.0, 1.0, secondTimePoint);
    EXPECT_EQ(secondState.timestamp, secondTimePoint);
    EXPECT_NEAR(secondState.accelerationEastWest, 1.0, 1e-12);
    EXPECT_NEAR(secondState.accelerationNorthSouth, 1.0, 1e-12);
    EXPECT_NEAR(secondState.speedEastWest, .001, 1e-12);
    EXPECT_NEAR(secondState.speedNorthSouth, .001, 1e-12);

    const auto thirdTimePoint = secondTimePoint + std::chrono::milliseconds(1);
    IMUUtils::KineticState thirdState = IMUUtils::Caclulate_Kinetic_Update(secondState, 2.0, -1.0, thirdTimePoint);
    EXPECT_EQ(thirdState.timestamp, thirdTimePoint);
    EXPECT_NEAR(thirdState.accelerationEastWest, 2.0, 1e-12);
    EXPECT_NEAR(thirdState.accelerationNorthSouth, -1.0, 1e-12);
    EXPECT_NEAR(thirdState.speedEastWest, .003, 1e-12);
    EXPECT_NEAR(thirdState.speedNorthSouth, 0.0, 1e-12);
}
