#pragma once

#include "../gps/gps_util.h"
#include "../navtypes.h"
#include "data.h"
#include "../kinematics/DiffDriveKinematics.h"
#include "../kinematics/DiffWristKinematics.h"

#include <optional>
#include <unordered_set>

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

/** @brief The current world interface being used. */
extern const WorldInterface WORLD_INTERFACE;

// TODO: add documentation
const DiffDriveKinematics& driveKinematics();
const DiffWristKinematics& wristKinematics();

/**
 * @brief Initialize the world interface.
 *
 * This method should only be called once, and must be called
 * before any other world interface methods.
 */
void world_interface_init();

/**
 * @brief Request the robot to drive at the given velocities.
 *
 * @param dtheta The heading velocity.
 * @param dx The forward velocity.
 * @return double If the requested velocities are too high, they will be scaled down.
 * The returned value is the scale divisor. If no scaling was performed, 1 is returned.
 */
double setCmdVel(double dtheta, double dx);

/**
 * @brief Get the velocity commanded to the robot. Depending on scaling,
 * may not be the same numbers passed to setCmdVel().
 *
 * @return std::pair<double, double> Pair of thetaVel, xVel.
 */
std::pair<double, double> getCmdVel();

/**
 * @brief Get the last measurement from the lidar.
 *
 * The returned points are in the robot's reference frame.
 *
 * @return types::DataPoint<navtypes::points_t> The lidar scan, if available.
 */
types::DataPoint<navtypes::points_t> readLidarScan();

/**
 * @brief Get the IDs of the currently supported cameras.
 *
 * @return The IDs of all cameras currently supported by the world interface, as a @ref
 * std::unordered_set.
 */
std::unordered_set<types::CameraID> getCameras();

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

/**
 * @brief Calculates the current transform in the frame of the starting pose based purely on
 * forward kinematics
 *
 * @return types::DataPoint<navtypes::transform_t> The current transform in the frame of the
 * starting pose. Generally, this datapoint should always be valid.
 */
types::DataPoint<navtypes::transform_t> readOdom();

/**
 * @brief Calculate the current pose velocity (in the robot's reference frame) using visual
 * odometry.
 *
 * @return types::DataPoint<navtypes::pose_t> The current velocity in each axis of the pose
 * vector, or an empty data point if no data is available.
 */
types::DataPoint<navtypes::pose_t> readVisualOdomVel();

/**
 * @brief Get the URC leg of the given index.
 *
 * @param index The index of the URC leg, in the range [0, 6]
 * @return navtypes::URCLeg Information for the URC leg of the given index.
 *
 * @deprecated This function is probably deprecated, and will be replaced in the future.
 */
navtypes::URCLeg getLeg(int index);

/**
 * @brief Set the robot indicator to indicate the given signal.
 *
 * @param signal The signal to display on the indicator.
 */
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

/**
 * @brief Get the last reported position of the specified motor.
 *
 * @param motor The motor to get the position from.
 * @return types::DataPoint<int32_t> The last reported position of the motor in millidegrees, 
 * if it exists. If the motor has not reported a position (because it hasn't been received yet
 * or if it doesn't have an encoder) then an empty data point is returned.
 */
types::DataPoint<int32_t> getMotorPos(robot::types::motorid_t motor);

using callbackid_t = unsigned long long;

callbackid_t addLimitSwitchCallback(
	robot::types::motorid_t motor,
	const std::function<void(
		robot::types::motorid_t motor,
		robot::types::DataPoint<robot::types::LimitSwitchData> limitSwitchData)>& callback);

void removeLimitSwitchCallback(callbackid_t id);

// TODO: document
void setJointPower(robot::types::jointid_t joint, double power);

/**
 * @param joint, the jointid_t of the joint to set the position of.
 * @param targetPos, the position to set the joint to, in millidegrees.
 */
void setJointPos(robot::types::jointid_t joint, int32_t targetPos);

/**
 * @param joint, the jointid_t of the joint to get the position of.
 * @return the position of the joint specified by the jointid_t argument joint,
 * in millidegrees.
 */
types::DataPoint<int32_t> getJointPos(robot::types::jointid_t joint);
// TODO: document
void zeroJoint(robot::types::jointid_t joint);
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
