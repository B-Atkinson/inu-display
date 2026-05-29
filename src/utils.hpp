/******************************************************************************
 * Filename:     utils.hpp
 *
 * Author:       Tran Sgt Brandon
 * Organization: Marine Corps Software Factory
 * Created On:   5/21/2026 1:38 PM
 * Description:  This header defines the namespace for IMU Coordinate Transformation.
 *
 ******************************************************************************/

#ifndef IMU_UTILS_HPP
#define IMU_UTILS_HPP

#pragma once

#include <cmath>
#include <stdexcept>

namespace IMUUtils {

/**
 * Converts Degrees to Radians from IMU readings.
 *
 * Expected behavior:
 *  - Ensures the heading is normalized to [0.0 -> 360.0] degrees
 *  - Returns heading in radians
 *
 * Assumptions:
 *  -Input is a valid heading in degrees (not NaN)
 *
 *  @param  degrees Heading angle in degrees
 *  @return         Normalized heading in radians from [0.0 -> 2 * pi]
 */
double DegreesToRadians(double degrees) {
  while (degrees >= 360.0) {
    degrees -= 360.0;
  }
  while (degrees < 0.0) {
    degrees += 360.0;
  }

  return degrees * M_PI / 180.0;
};

/**
 *
 * @brief Converts to Global X Acceleration from IMU (x,y) readings.
 *
 * @remarks
 * Expected behavior:
 * - Returns linear acceleration in global X (East-positive) in meters / second / second.
 * - Uses same rotation formula as defined in global_rotation_tests.cpp
 * - boat_x and boat_y are accelerations in boat frame (right/forward)
 *
 * Assumptions:
 * - Assumes theta_t is measured with respect to True North.
 *
 * @param   theta_t The heading of the boat (which direction the boat's
 *                  y-axis is pointing) in radians measured from True North.
 * @param   boat_x  The boat's linear acceleration in the x-axis in meters per second (m/s).
 * @param   boat_y  The boat's linear acceleration in the y-axis in meters per second (m/s).
 * @return          Linear acceleration in global x axis (East/West with East being positive) measured in meters per sec
 * per second.
 */
double InertialToGlobal_X(double theta_t, double boat_x, double boat_y) {
  double global_X = std::cos(theta_t) * boat_x + std::sin(theta_t) * boat_y;
  return global_X;
};

/**
 *  @brief Converts to Global Y Acceleration from IMU (x,y) readings.
 *
 * @remark
 * Expected behavior:
 * - Returns linear acceleration in global Y (North-positive) in meters / second / second.
 * - Uses the same rotation formula as defined in global_rotation_tests.cpp
 *
 * @remark
 * Assumptions:
 * - theta_t is heading in radians measured from True North
 * - boat_x and boat_y are accelerations in boat frame (right/forward)
 *
 * @param   theta_t Boat heading in radians from True North.
 * @param   boat_x  The boat's linear acceleration in the x-axis in meters per second (m/s).
 * @param   boat_y  The boat's linear acceleration in the y-axis in meters per second(m/s).
 * @return          Linear acceleration in global y axis (North/South with North being positive) measured in meters per
 * sec per second.
 *
 */
double InertialToGlobal_Y(double theta_t, double boat_x, double boat_y) {
  double global_Y = std::cos(theta_t) * boat_y - std::sin(theta_t) * boat_x;
  return global_Y;
};

/**
 * @brief Converts Global X acceleration to acceleration in global X axis.
 *
 * @remark
 * Expected behavior:
 * - Returns linear acceleration in the x axis in meters per second per second (East/West).
 *
 * @remark
 * Assumptions:
 * - Assumes the conversion rate for 1 degree latitude is 111,111.11 meters.
 * - Assumes the conversion rate for 1 degree longitude is 111,111.11 * cos(latitude).
 *
 * @param   boat_latitude   A latitude position (measures how far North/South from the equator with North being postive)
 * measured in degrees.
 * @param   boat_longitude  A longitude position (measures how far East/West from the prime meridian with East being
 * positive) measures in degrees)
 * @param   global_x        The boat's linear acceleration in the x axis in meters per second per second.
 * @return                  Linear acceleration in global x axis (East/West with East being positive) measured in
 * degrees longitude per second per second.
 */
double Convert_Global_X_To_DegPerS2(double boat_latitude, double boat_longitude, double global_x) {
  double meters_per_deg_longitude = 111111.11 * std::cos(boat_latitude * M_PI / 180.0);
  double accel_longitude = global_x / meters_per_deg_longitude;

  return accel_longitude;
};

/**
 * @brief Converts Global Y acceleration to acceleration in global Y axis.
 *
 * @remark
 * Expected behavior:
 * - Returns linear acceleration in the y axis in meters per second per second (North/South).
 *
 * @remark
 * Assumptions:
 * - Assumes the conversion rate for 1 degree latitude is 111,111.11 meters.
 *
 * @param   boat_latitude   A latitude position (measures how far North/South from the equator with North being postive)
 * measured in degrees.
 * @param   boat_longitude  A longitude position (measures how far East/West from the prime meridian with East being
 * positive) measures in degrees)
 * @param   global_y        The boat's linear acceleration in the y axis in meters per second per second.
 * @return                  Linear acceleration in global y axis (North/South with North being positive) measured in
 * degrees latitude per second per second.
 */
double Convert_Global_Y_to_DegPerS2(double boat_latitude, double boat_longitude, double global_y) {
  double accel_latitude = global_y / 111111.11;

  return accel_latitude;
};

/**
 * @brief Converts quaternions w + (i,j,k) to calculate magnetic heading.
 *
 * @remark
 * Expected Behavior:
 * The returned heading will always be between [0.0, 360).
 *
 * @remark
 * Assumptions:
 * - w, i, j and k values are normalized.
 *
 * @param w Scalar
 * @param i Quaternion rotation with respect to x axis.
 * @param j Quaternion rotation with respect to y axis.
 * @param k Quaternion rotation with respect to z axis.
 * @return  A double containing the magnetic heading of the IMU in degrees (not radians).
 *
 */
double Calculate_Magnetic_Heading(double w, double i, double j, double k) {

  double magnetic_heading = std::atan2(2 * ((w * j) + (i * k)), 1 - 2 * ((j * j) + (k * k)));

  return magnetic_heading * 180 / M_PI;
};
}; // namespace IMUUtils

#endif // IMU_UTILS_HPP