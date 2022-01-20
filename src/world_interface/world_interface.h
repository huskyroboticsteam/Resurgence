#pragma once

#include "../simulator/utils.h"
#include "data.h"
#include "../gps/gps_util.h"

#include <optional>

// forward declare cam::CameraParams instead of including it
// we do this to avoid unnecessarily including OpenCV in all build targets
namespace cam {
class CameraParams;
}

// Call this before trying to do anything else
void world_interface_init();

// If the requested dtheta/dx is too fast for the robot to execute, it will
// scale them down and return the corresponding scale factor.
double setCmdVel(double dtheta, double dx);

std::pair<double, double> getCmdVel();

// read measurement from the lidar
DataPoint<points_t> readLidarScan();

/**
 * @brief Check if a new camera frame from the specified camera is available.
 *
 * @param camera The ID of the camera to check
 * @param oldFrameNum The frame number of the old frame. A camera frame is "new" if its id is
 * different than this.
 * @returns true iff a new camera frame is available.
 */
bool hasNewCameraFrame(CameraID camera, uint32_t oldFrameNum);

/**
 * @brief Read the latest frame from the given camera. This is not guaranteed to change between
 * calls, so use hasNewCameraFrame() to check if a new frame is available.
 *
 * @param camera The ID of the camera to read from.
 * @return DataPoint<CameraFrame> A datapoint containing the latest frame.
 * If an error occurs or the ID is invalid, returns an invalid datapoint.
 */
DataPoint<CameraFrame> readCamera(CameraID camera);

/**
 * @brief Get the intrinsic params of the specified camera, if it exists.
 *
 * @param camera The ID of the camera for which to get the intrinsic params.
 * @returns The intrinsic params if it exists, else an empty optional object.
 */
std::optional<cam::CameraParams> getCameraIntrinsicParams(CameraID camera);

/**
 * @brief Get the extrinsic params of the specified camera, if it exists.
 *
 * @param camera The ID of the camera for which to get the extrinsic params.
 * @returns The extrinsic params if it exists, else an empty optional object.
 */
std::optional<cv::Mat> getCameraExtrinsicParams(CameraID camera);

/**
 * @brief Read measurement from the CV system. As of now, returns a vector of fixed length, one
 * for each post in the competition.
 *
 * @return DataPoint<points_t> A vector of fixed lengths. Non-visible markers are denoted with
 * {0,0,0}, while all nonzero points are visible marker. The index of a landmark in this vector
 * is its id.
 */
landmarks_t readLandmarks();

// read from the sensor and return unmodified lat/long coordinates
// this should be implemented by hardware-specific code and should not be
// used by client code
DataPoint<gpscoords_t> readGPS_private();

// returns true iff the GPS has a fix
bool gpsHasFix();

// Get the current position in the global frame based on a GPS measurement.
// Note that these values are NOT lat/long.
DataPoint<point_t> readGPS();

// Given the current longitude and latitude, convert to a point_t position representation
// If the GPS has not acquired a fix yet, return an empty optional
std::optional<point_t> gpsToMeters(const gpscoords_t &coord);

// Calculates the current transform in the global frame based purely on forward kinematics
DataPoint<transform_t> readOdom();

// Calculate the current pose velocity (in the robot's reference frame) using visual odometry.
DataPoint<pose_t> readVisualOdomVel();

// `index` must be in the range 0-6 (the URC competition will have 7 legs)
URCLeg getLeg(int index);

// set the indicator to indicate the given signal. May no-op if indicator is not supported.
void setIndicator(indication_t signal);

/**
 * @brief Set the PWM command of the given motor.
 *
 * @param motor The name of the motor. Must be a valid motor name.
 * @param normalizedPWM The PWM command, in the range [-1, 1]
 */
void setMotorPWM(const std::string& motor, double normalizedPWM);

/**
 * @brief Set the target position of the motor.
 *
 * @param motor The name of the motor. Must be a valid motor name.
 * @param targetPos The target position. Refer to the specific motor for more information.
 */
void setMotorPos(const std::string& motor, int32_t targetPos);
