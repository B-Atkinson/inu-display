/******************************************************************************
 * Filename:     imu_utils_tests.cpp
 *
 * Author:       Tran Sgt Brandon, Gomez LCpl Greg
 * Organization: Marine Corps Software Factory
 * Created On:   5/21/2026 2:45 PM
 * Description:  This test files validates (1) DegreesToRadians, (2)
 * InertialToGlobal_X and (3) InertialToGlobal_Y functionality.
 *
 ******************************************************************************/

#include "utils.hpp"
#include <cmath>
#include <gmock/gmock.h>
#include <gtest/gtest.h>

using namespace IMUUtils;

TEST(IMUUtils, Convert_Degrees_To_Radians) {

  struct Case {
    double degrees_input;
    double degrees_expected;
  };

  const std::vector<Case> cases = {// These tests are pass correct
                                   {-45.0, M_PI * 7 / 4}, {0.0, M_PI * 0}, {45.0, M_PI / 4},      {90.0, M_PI / 2},
                                   {135.0, M_PI * 3 / 4}, {180.0, M_PI},   {225.0, M_PI * 5 / 4}, {270.0, M_PI * 3 / 2},
                                   {315.0, M_PI * 7 / 4}, {360.0, 0.0},    {405.0, M_PI / 4}
  };

  for (const auto &c : cases) {
    EXPECT_NEAR(IMUUtils::DegreesToRadians(c.degrees_input), c.degrees_expected, 1e-9);
  };
};
// -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
// -- -- -- -- -- -- -- -- -- -- -- -- -
TEST(IMUUtils, Convert_Local_XY_Acceleration_To_Global_X_Acceleration) {
  struct Case {
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
      {IMUUtils::DegreesToRadians(405.0), 1.0, 1.0, std::sqrt(2.0)}
  };

  for (const auto &c : cases) {
    EXPECT_NEAR(InertialToGlobal_X(c.theta_input, c.x_accel_input, c.y_accel_input), c.x_accel_output, 1e-9);
  };
};
// ----------------------------------------------------------------------------
TEST(IMUUtils, Convert_Local_xy_Acceleration_To_Global_Y_Acceleration) {
  struct Case {
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
      {IMUUtils::DegreesToRadians(405.0), 1.0, 1.0, 0.0}
  }; // cases

  for (const auto &c : cases) {
    EXPECT_NEAR(InertialToGlobal_Y(c.theta_input, c.x_accel_input, c.y_accel_input), c.y_accel_output, 1e-9);
  };
};
// ----------------------------------------------------------------------------
TEST(IMUUtils, Convert_Global_X_Accel_Into_Degrees_Longitude) {
  struct Case {
    double degree_latitude_input;
    double degree_longitude_input;
    double accel_x_input;
    double accel_deg_longitude_output;
  };

  const std::vector<Case> cases = {
      // Near equator at +1 m/s2
      {0.0, 0.0, 1.0, 1.0 / (111111.11 * std::cos(0.0 * M_PI / 180.0))},
      // 45 deg latitude at +1 m/s2
      {45.0, 0.0, 1.0, 1.0 / (111111.11 * std::cos(45.0 * M_PI / 180.0))},
      // 60 deg latitude at +1 m/s2
      {60.0, 0.0, 1.0, 1.0 / (111111.11 * std::cos(60.0 * M_PI / 180.0))},
      // Near equator at -1 m/s2
      {0.0, 0.0, -1.0, -1.0 / (111111.11 * std::cos(0.0 * M_PI / 180.0))},
      // 45 deg latitude at -1 m/s2
      {45.0, 0.0, -1.0, -1.0 / (111111.11 * std::cos(45.0 * M_PI / 180.0))},
      // 60 deg latitude at -1 m/s2
      {60.0, 0.0, -1.0, -1.0 / (111111.11 * std::cos(60.0 * M_PI / 180.0))}
  };

  for (const auto &c : cases) {
    EXPECT_NEAR(
        IMUUtils::Convert_Global_X_To_DegPerS2(c.degree_latitude_input, c.degree_longitude_input, c.accel_x_input),
        c.accel_deg_longitude_output, 1e-9
    );
  };
};

// ----------------------------------------------------------------------------
TEST(IMUUtils, Convert_Global_Y_Accel_Into_Degrees_Latitude) {
  struct Case {
    double degree_latitude_input;
    double degree_longitude_input;
    double accel_y_input;
    double accel_deg_latitude_output;
  };

  const std::vector<Case> cases = {
      // Near equator at +1 m/s2
      {0.0, 0.0, 1.0, 1.0 / 111111.11},
      // 45 deg latitude at +1 m/s2
      {0.0, 45.0, 1.0, 1.0 / 111111.11},
      // 60 deg latitude at +1 m/s2
      {0.0, 60.0, 1.0, 1.0 / 111111.11},
      // Near equator at -1 m/s2
      {0.0, 0.0, -1.0, -1.0 / 111111.11},
      // 45 deg latitude at -1 m/s2
      {0.0, 45.0, -1.0, -1.0 / 111111.11},
      // 60 deg latitude at -1 m/s2
      {0.0, 60.0, -1.0, -1.0 / 111111.11}
  };

  for (const auto &c : cases) {
    EXPECT_NEAR(
        IMUUtils::Convert_Global_X_To_DegPerS2(c.degree_latitude_input, c.degree_longitude_input, c.accel_y_input),
        c.accel_deg_latitude_output, 1e-9
    );
  };
};
// ----------------------------------------------------------------------------
TEST(IMUUtils, Calculating_Magnetic_Heading_From_Rotation_Vectors) {
  struct Case {
    double w_input;
    double i_input;
    double j_input;
    double k_input;
    double magnetic_heading_output;
  };

  const std::vector<Case> cases = {
      {0.0, 0.0, 0.0, 0.0, 0.0},
      {1.0, 0.0, 0.0, 0.0, 0.0},

      // {0.0, std::sqrt(2) / 2, std::sqrt(2) / 2, 0.0, 180.0},
      // {0.0, -std::sqrt(2) / 2, std::sqrt(2) / 2, 0.0, 180.0},
      // {0.0, -std::sqrt(2) / 2, -std::sqrt(2) / 2, 0.0, -180.0},
      // {0.0, std::sqrt(2) / 2, -std::sqrt(2) / 2, 0.0, 180.0},

      // {0.0, std::sqrt(3) / 3, std::sqrt(3) / 3, std::sqrt(3) / 3, 116.565051177},
      // {0.0, -std::sqrt(3) / 3, std::sqrt(3) / 3, std::sqrt(3) / 3, -116.565051177},
      // {0.0, -std::sqrt(3) / 3, -std::sqrt(3) / 3, std::sqrt(3) / 3, -116.565051177},
      // {0.0, std::sqrt(3) / 3, -std::sqrt(3) / 3, std::sqrt(3) / 3, 116.565051177},

      // {0.0, std::sqrt(3) / 3, std::sqrt(3) / 3, -std::sqrt(3) / 3, -116.565051177},
      // {0.0, -std::sqrt(3) / 3, std::sqrt(3) / 3, -std::sqrt(3) / 3, 116.565051177},
      // {0.0, -std::sqrt(3) / 3, -std::sqrt(3) / 3, -std::sqrt(3) / 3, 116.565051177},
      // {0.0, std::sqrt(3) / 3, -std::sqrt(3) / 3, -std::sqrt(3) / 3, -116.565051177},

      // {std::sqrt(4) / 4, std::sqrt(4) / 4, std::sqrt(4) / 4, std::sqrt(4) / 4, 90.0},
      // {std::sqrt(4) / 4, -std::sqrt(4) / 4, std::sqrt(4) / 4, std::sqrt(4) / 4, 0.0},
      // {std::sqrt(4) / 4, -std::sqrt(4) / 4, -std::sqrt(4) / 4, std::sqrt(4) / 4, -90.0},
      // {std::sqrt(4) / 4, std::sqrt(4) / 4, -std::sqrt(4) / 4, std::sqrt(4) / 4, 0.0},

      // {std::sqrt(4) / 4, std::sqrt(4) / 4, std::sqrt(4) / 4, -std::sqrt(4) / 4, 0.0},
      // {std::sqrt(4) / 4, -std::sqrt(4) / 4, std::sqrt(4) / 4, -std::sqrt(4) / 4, 90.0},
      // {std::sqrt(4) / 4, -std::sqrt(4) / 4, -std::sqrt(4) / 4, -std::sqrt(4) / 4, 0.0},
      // {std::sqrt(4) / 4, std::sqrt(4) / 4, -std::sqrt(4) / 4, -std::sqrt(4) / 4, -90.0},

      // {-std::sqrt(4) / 4, std::sqrt(4) / 4, std::sqrt(4) / 4, std::sqrt(4) / 4, 0.0},
      // {-std::sqrt(4) / 4, -std::sqrt(4) / 4, std::sqrt(4) / 4, std::sqrt(4) / 4, -90.0},
      // {-std::sqrt(4) / 4, -std::sqrt(4) / 4, -std::sqrt(4) / 4, std::sqrt(4) / 4, 0.0},
      // {-std::sqrt(4) / 4, std::sqrt(4) / 4, -std::sqrt(4) / 4, std::sqrt(4) / 4, 90.0},

      // {-std::sqrt(4) / 4, std::sqrt(4) / 4, std::sqrt(4) / 4, -std::sqrt(4) / 4, -90.0},
      // {-std::sqrt(4) / 4, -std::sqrt(4) / 4, std::sqrt(4) / 4, -std::sqrt(4) / 4, 0.0},
      // {-std::sqrt(4) / 4, -std::sqrt(4) / 4, -std::sqrt(4) / 4, -std::sqrt(4) / 4, 90.0},
      // {-std::sqrt(4) / 4, std::sqrt(4) / 4, -std::sqrt(4) / 4, -std::sqrt(4) / 4, 0.0},

      // Small i, j, k rotation about <0, 1, 0> vector.
      {std::cos(10.0 * M_PI / 180.0 / 2), std::sin(10.0 * M_PI / 180.0 / 2) * 0 / std::sqrt(0 * 0 + 0 * 0 + 1 * 1),
       std::sin(10.0 * M_PI / 180.0 / 2) * 1 / std::sqrt(0 * 0 + 0 * 0 + 1 * 1),
       std::sin(10.0 * M_PI / 180.0 / 2) * 0 / std::sqrt(0 * 0 + 0 * 0 + 1 * 1), 10.0},

      // Small i, j, k rotation about <0, 0, 1> vector.
      {std::cos(10.0 * M_PI / 180.0 / 2), std::sin(10.0 * M_PI / 180.0 / 2) * 0 / std::sqrt(0 * 0 + 0 * 0 + 1 * 1),
       std::sin(10.0 * M_PI / 180.0 / 2) * 0 / std::sqrt(0 * 0 + 0 * 0 + 1 * 1),
       std::sin(10.0 * M_PI / 180.0 / 2) * 1 / std::sqrt(0 * 0 + 0 * 0 + 1 * 1), 0.0},

      // Small i, j, k rotation about <1, 0, 0> vector.
      {std::cos(10.0 * M_PI / 180.0 / 2), std::sin(10.0 * M_PI / 180.0 / 2) * 1 / std::sqrt(1 * 1 + 0 * 0 + 0 * 0),
       std::sin(10.0 * M_PI / 180.0 / 2) * 0 / std::sqrt(1 * 1 + 0 * 0 + 0 * 0),
       std::sin(10.0 * M_PI / 180.0 / 2) * 0 / std::sqrt(1 * 1 + 0 * 0 + 0 * 0), 0.0},

      // Small i, j, k rotation about <0, 0, -1> vector.
      {std::cos(10.0 * M_PI / 180.0 / 2), std::sin(10.0 * M_PI / 180.0 / 2) * -1 / std::sqrt(-1 * -1 + 0 * 0 + 0 * 0),
       std::sin(10.0 * M_PI / 180.0 / 2) * 0 / std::sqrt(1 * 1 + 0 * 0 + 0 * 0),
       std::sin(10.0 * M_PI / 180.0 / 2) * 0 / std::sqrt(1 * 1 + 0 * 0 + 0 * 0), 0.0},

      // Small i, j, k rotation about <1, 1, 1> vector.
      {std::cos(10.0 * M_PI / 180.0 / 2), std::sin(10.0 * M_PI / 180.0 / 2) * 1 / std::sqrt(1 * 1 + 1 * 1 + 1 * 1),
       std::sin(10.0 * M_PI / 180.0 / 2) * 1 / std::sqrt(1 * 1 + 1 * 1 + 1 * 1),
       std::sin(10.0 * M_PI / 180.0 / 2) * 1 / std::sqrt(1 * 1 + 1 * 1 + 1 * 1), 6.07328001016},

      // Small i, j, k rotation about <-1, 1, 1> vector.
      {std::cos(10.0 * M_PI / 180.0 / 2), std::sin(10.0 * M_PI / 180.0 / 2) * -1 / std::sqrt(-1 * -1 + 1 * 1 + 1 * 1),
       std::sin(10.0 * M_PI / 180.0 / 2) * 1 / std::sqrt(-1 * -1 + 1 * 1 + 1 * 1),
       std::sin(10.0 * M_PI / 180.0 / 2) * 1 / std::sqrt(-1 * -1 + 1 * 1 + 1 * 1), 5.49299876217},

      // Small i, j, k rotation about <-1, -1, 1> vector.
      {std::cos(10.0 * M_PI / 180.0 / 2), std::sin(10.0 * M_PI / 180.0 / 2) * -1 / std::sqrt(-1 * -1 + -1 * -1 + 1 * 1),
       std::sin(10.0 * M_PI / 180.0 / 2) * -1 / std::sqrt(-1 * -1 + -1 * -1 + 1 * 1),
       std::sin(10.0 * M_PI / 180.0 / 2) * 1 / std::sqrt(-1 * -1 + -1 * -1 + 1 * 1), -6.07328001016},

      // Small i, j, k rotation about <1, -1, 1> vector.
      {std::cos(10.0 * M_PI / 180.0 / 2), std::sin(10.0 * M_PI / 180.0 / 2) * 1 / std::sqrt(1 * 1 + -1 * -1 + 1 * 1),
       std::sin(10.0 * M_PI / 180.0 / 2) * -1 / std::sqrt(1 * 1 + -1 * -1 + 1 * 1),
       std::sin(10.0 * M_PI / 180.0 / 2) * 1 / std::sqrt(1 * 1 + -1 * -1 + 1 * 1), -5.49299876217},

      // Small i, j, k rotation about <1, 1, -1> vector.
      {std::cos(10.0 * M_PI / 180.0 / 2), std::sin(10.0 * M_PI / 180.0 / 2) * 1 / std::sqrt(1 * 1 + 1 * 1 + -1 * -1),
       std::sin(10.0 * M_PI / 180.0 / 2) * 1 / std::sqrt(1 * 1 + 1 * 1 + -1 * -1),
       std::sin(10.0 * M_PI / 180.0 / 2) * -1 / std::sqrt(1 * 1 + 1 * 1 + -1 * -1), 5.49299876217},
      // Small i, j, k rotation about <-1, 1, -1> vector.
      {std::cos(10.0 * M_PI / 180.0 / 2), std::sin(10.0 * M_PI / 180.0 / 2) * -1 / std::sqrt(-1 * -1 + 1 * 1 + -1 * -1),
       std::sin(10.0 * M_PI / 180.0 / 2) * 1 / std::sqrt(-1 * -1 + 1 * 1 + -1 * -1),
       std::sin(10.0 * M_PI / 180.0 / 2) * -1 / std::sqrt(-1 * -1 + 1 * 1 + -1 * -1), 6.07328001016},
      // Small i, j, k rotation about <-1, -1, -1> vector.
      {std::cos(10.0 * M_PI / 180.0 / 2), std::sin(10.0 * M_PI / 180.0 / 2) * -1 / std::sqrt(-1 * -1 + -1 * -1 + -1 * -1),
       std::sin(10.0 * M_PI / 180.0 / 2) * -1 / std::sqrt(-1 * -1 + -1 * -1 + -1 * -1),
       std::sin(10.0 * M_PI / 180.0 / 2) * -1 / std::sqrt(-1 * -1 + -1 * -1 + -1 * -1), -5.49299876217},
      // Small i, j, k rotation about <1,-1, -1> vector.
      {std::cos(10.0 * M_PI / 180.0 / 2), std::sin(10.0 * M_PI / 180.0 / 2) * 1 / std::sqrt(1 * 1 + -1 * -1 + -1 * -1),
       std::sin(10.0 * M_PI / 180.0 / 2) * -1 / std::sqrt(1 * 1 + -1 * -1 + -1 * -1),
       std::sin(10.0 * M_PI / 180.0 / 2) * -1 / std::sqrt(1 * 1 + -1 * -1 + -1 * -1), -6.07328001016},

  };

  for (const auto &c : cases) {
    EXPECT_NEAR(IMUUtils::Calculate_Magnetic_Heading(c.w_input, c.i_input, c.j_input, c.k_input), c.magnetic_heading_output, 1e-6);
  };
};
// ----------------------------------------------------------------------------
// TEST(IMUUtils, Calculating_Magnetic_Heading_From_Rotation_Vectors_Throws_Exception) {
//   EXPECT_THROW(IMUUtils::Calculate_Magnetic_Heading(10.0, 1.0, 1.0, 1.0), std::runtime_error);
//   EXPECT_NO_THROW(IMUUtils::Calculate_Magnetic_Heading(1.0, 0.0, 0.0, 0.0));
// };