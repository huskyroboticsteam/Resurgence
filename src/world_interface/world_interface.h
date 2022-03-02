#pragma once

#include "../gps/gps_util.h"
#include "../navtypes.h"
#include "data.h"

#include <optional>

// forward declare cam::CameraParams instead of including it
// we do this to avoid unnecessarily including OpenCV in all build targets
namespace cam {
class CameraParams;
}

/**
 * @namespace robot
 * @brief Collection of functions that allows interfacing with the hardware and the world.
 */
namespace robot {

/**
 * @brief An enum which defines the possible types of world interfaces.
 *
 * @see WORLD_INTERFACE
 */
enum class WorldInterface { real, sim2d, sim3d, noop };

/**
 * @brief The current world interface being used.
 */
extern const WorldInterface WORLD_INTERFACE;

// Call this before trying to do anything else
void world_interface_init();

// If the requested dtheta/dx is too fast for the robot to execute, it will
// scale them down and return the corresponding scale divisor.
double setCmdVel(double dtheta, double dx);

/**
 * @brief Get the velocity commanded to the robot. Depending on scaling,
 * may not be the same numbers passed to setCmdVel().
 *
 * @return std::pair<double, double> Pair of thetaVel, xVel.
 */
std::pair<double, double> getCmdVel();

// read measurement from the lidar
types::DataPoint<navtypes::points_t> readLidarScan();

/**
 * @brief Check if a new camera frame from the specified camera is available.
 *
 * @param camera The ID of the camera to check
 * @param oldFrameNum The frame number of the old frame. A camera frame is "new" if its id is
 * different than this.
 * @returns true iff a new camera frame is available.
 */
bool hasNewCameraFrame(types::CameraID camera, uint32_t oldFrameNum);

/**
 * @brief Read the latest frame from the given camera. This is not guaranteed to change between
 * calls, so use hasNewCameraFrame() to check if a new frame is available.
 *
 * @param camera The ID of the camera to read from.
 * @return DataPoint<CameraFrame> A datapoint containing the latest frame.
 * If an error occurs or the ID is invalid, returns an invalid datapoint.
 */
types::DataPoint<types::CameraFrame> readCamera(types::CameraID camera);

/**
 * @brief Get the intrinsic params of the specified camera, if it exists.
 *
 * @param camera The ID of the camera for which to get the intrinsic params.
 * @returns The intrinsic params if it exists, else an empty optional object.
 */
std::optional<cam::CameraParams> getCameraIntrinsicParams(types::CameraID camera);

/**
 * @brief Get the extrinsic params of the specified camera, if it exists.
 *
 * @param camera The ID of the camera for which to get the extrinsic params.
 * @returns The extrinsic params if it exists, else an empty optional object.
 */
std::optional<cv::Mat> getCameraExtrinsicParams(types::CameraID camera);

/**
 * @brief Read measurement from the CV system. As of now, returns a vector of fixed length, one
 * for each post in the competition.
 *
 * @return DataPoint<points_t> A vector of fixed lengths. Non-visible markers are denoted with
 * {0,0,0}, while all nonzero points are visible marker. The index of a landmark in this vector
 * is its id.
 */
types::landmarks_t readLandmarks();

/**
 * @brief Check if the GPS sensor has acquired a fix.
 *
 * @return true iff the GPS has a fix
 */
bool gpsHasFix();

/**
 * @brief Get the current position in the global map frame based on a GPS measurement.
 * Note that these values are NOT lat/long. Note that for the map frame, +x=+lat,+y=-lon.
 *
 * @return DataPoint<point_t> The last GPS reading, in the map space.
 */
types::DataPoint<navtypes::point_t> readGPS();

/**
 * @brief Read the current heading in the global map frame based on an IMU measurement.
 * Note that the heading is in radians, CCW, and 0=North.
 *
 * @return DataPoint<double> The last heading reading, or none if measurement not available.
 */
types::DataPoint<double> readIMUHeading();

/**
 * @brief Get the ground truth pose, if available. This is only available in simulation.
 *
 * @return DataPoint<pose_t> The ground truth pose, or an empty datapoint if unavailable.
 */
types::DataPoint<navtypes::pose_t> getTruePose();

/**
 * @brief Convert GPS coordinates into a coordinate on the map frame.
 *
 * @param coord The coord to transform into map space.
 * @return std::optional<point_t> The transformed point, or an empty datapoint
 * if the GPS does not have a fix.
 */
std::optional<navtypes::point_t> gpsToMeters(const navtypes::gpscoords_t& coord);

// Calculates the current transform in the global frame based purely on forward kinematics
types::DataPoint<navtypes::transform_t> readOdom();

// Calculate the current pose velocity (in the robot's reference frame) using visual odometry.
types::DataPoint<navtypes::pose_t> readVisualOdomVel();

// `index` must be in the range 0-6 (the URC competition will have 7 legs)
navtypes::URCLeg getLeg(int index);

// set the indicator to indicate the given signal. May no-op if indicator is not supported.
void setIndicator(types::indication_t signal);

/**
 * @brief Set the PWM command of the given motor.
 *
 * @param motor The motor to set the PWM of.
 * @param power The power command, in the range [-1, 1]
 */
void setMotorPower(robot::types::motorid_t motor, double power);

/**
 * @brief Set the target position of the motor. This will have no effect if the motor
 * does not support PID.
 *
 * @param motor The motor to set the target position of.
 * @param targetPos The target position, in millidegrees. Refer to the specific motor for more
 * information.
 */
void setMotorPos(robot::types::motorid_t motor, int32_t targetPos);

} // namespace robot

namespace gps {
/**
 * @brief read from the sensor and return unmodified lat/long coordinates
 * this should be implemented by hardware-specific code. (readGPS() should NOT be overridden)
 *
 * @return DataPoint<gpscoords_t> The last GPS coordinates measured by the sensor, or empty if
 * no fix.
 */
robot::types::DataPoint<navtypes::gpscoords_t> readGPSCoords();
} // namespace gps
