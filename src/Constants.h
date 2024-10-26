#pragma once

#include "utils/time.h"
#include "world_interface/data.h"

#include <array>
#include <chrono>
#include <cmath>
#include <string>

#include <frozen/unordered_map.h>

namespace Constants {
// TODO: make sure these are still accurate with the new arm.
extern const double SHOULDER_LENGTH;
extern const double ELBOW_LENGTH;

/**
   Number of millidegrees per degree
 */
extern const float MDEG_PER_DEG;

// TODO: tune these drive extern constants
extern const double ROBOT_LENGTH;
extern const double ROBOT_WIDTH;
/**
   Distance between left and right wheels.
*/
extern const double WHEEL_BASE;
/**
  Effective distance between wheels. Tweaked so that actual rover angular rate roughly matches
  the commanded angular rate.
 */
extern const double EFF_WHEEL_BASE;
extern const double WHEEL_RADIUS;
extern const double PWM_PER_RAD_PER_SEC;
extern const double MAX_DRIVE_PWM;

/**
   @brief Maximum tangential velocity for the rover's wheels.

   If the rover is driving straight and not turning, this is the maximum forward velocity
   (i.e. `dx` in setCmdVel()) of the rover.
 */
extern const double MAX_WHEEL_VEL;
/**
   @brief Maximum angular velocity (i.e. `dtheta` in setCmdVel()) of the rover.

   Computed assuming that the left wheel is going at full speed backwards while the right wheel
   is going at full speed forwards.
 */
extern const double MAX_DTHETA;

// TODO: We need to recalibrate the camera, since we replaced it with a different one.
// TODO: rename cameras (in MC as well) as appropriate
extern const char* MAST_CAMERA_CONFIG_PATH;
extern const robot::types::CameraID MAST_CAMERA_ID;

extern const char* FOREARM_CAMERA_CONFIG_PATH;
extern const robot::types::CameraID FOREARM_CAMERA_ID;

extern const char* HAND_CAMERA_CONFIG_PATH;
extern const robot::types::CameraID HAND_CAMERA_ID;

extern const uint16_t WS_SERVER_PORT;

/**
   WebSocket server endpoint for the mission control protocol.
 */
extern const char* MC_PROTOCOL_NAME;
/**
   WebSocket server endpoint for the simulator protocol.
 */
extern const char* SIM_PROTOCOL_NAME;
/**
   WebSocket server endpoint for the DGPS protocol.
 */
extern const char* DGPS_PROTOCOL_NAME;
/**
   Websocket server endpoint for ArduPilot protocol
 */
extern const char* ARDUPILOT_PROTOCOL_NAME;

extern const std::chrono::milliseconds JOINT_POWER_REPEAT_PERIOD;
extern const std::chrono::milliseconds ARM_IK_UPDATE_PERIOD;

namespace Drive {
// Represents the allowable error in millidegrees for steer motors to still process a drive
// request. That is, we make sure all the wheels are close enough to their target rotation
// before actually driving.
extern const double STEER_EPSILON;
} // namespace Drive

namespace Nav {
// Distance (m) we could have traveled forward in the time it takes to turn 1 radian
extern const double RADIAN_COST;
// Planner stays this far away from obstacles (m)
extern const double SAFE_RADIUS;
extern const int MAX_ITERS;
extern const double PLAN_RESOLUTION;
extern const double SEARCH_RADIUS_INCREMENT;
extern const double GPS_WAYPOINT_RADIUS;
extern const double LANDMARK_WAYPOINT_RADIUS;
extern const double EPS;
} // namespace Nav

// Lidar
namespace Lidar {
extern const std::string RP_PATH;
extern const double MM_PER_M;
extern const uint32_t RPLIDAR_A1_BAUDRATE;
extern const uint32_t RPLIDAR_S1_BAUDRATE;
} // namespace Lidar

// Video encoding
namespace video {
/**
 * @brief Default RF constant for H264 streams, if not specified in STREAM_RFS.
 *
 * In the range [1, 51], higher values means more compression.
 * Should not be below 21, since quality is basically the same but
 * bandwidth is much higher.
 */
extern const int H264_RF_CONSTANT;

/**
 * @brief Stream-specific RF constants.
 */
extern const std::unordered_map<robot::types::CameraID, int> STREAM_RFS;
} // namespace video

/**
 * A map that pairs each of the joints to its corresponding motor.
 * (one-to-one pairs only)
 */
constexpr auto JOINT_MOTOR_MAP =
	frozen::make_unordered_map<robot::types::jointid_t, robot::types::motorid_t>(
		{{robot::types::jointid_t::frontLeftSwerve, robot::types::motorid_t::frontLeftSwerve},
		 {robot::types::jointid_t::frontRightSwerve,
		  robot::types::motorid_t::frontRightSwerve},
		 {robot::types::jointid_t::rearLeftSwerve, robot::types::motorid_t::rearLeftSwerve},
		 {robot::types::jointid_t::rearRightSwerve, robot::types::motorid_t::rearRightSwerve},
		 {robot::types::jointid_t::armBase, robot::types::motorid_t::armBase},
		 {robot::types::jointid_t::shoulder, robot::types::motorid_t::shoulder},
		 {robot::types::jointid_t::elbow, robot::types::motorid_t::elbow},
		 {robot::types::jointid_t::forearm, robot::types::motorid_t::forearm},
		 {robot::types::jointid_t::hand, robot::types::motorid_t::hand},
		 {robot::types::jointid_t::activeSuspension,
		  robot::types::motorid_t::activeSuspension}});

// Arm inverse kinematics
namespace arm {
/**
 * Percentage of fully extended overall arm length to limit arm extension within.
 */
extern const double SAFETY_FACTOR;

/**
 * Maximum commanded end-effector velocity, in m/s
 */
extern const double MAX_EE_VEL;
extern const double IK_SOLVER_THRESH;

extern const int IK_SOLVER_MAX_ITER;

/**
 * The joints corresponding to the motors used for IK in the arm. The ordering in this array is
 * the canonical ordering of these joints for IK purposes.
 */
extern const std::array<robot::types::jointid_t, 2> IK_MOTOR_JOINTS;

/**
 * The motors used in IK. The i-th element in this array corresponds to the joint in the i-th
 * element of `IK_MOTOR_JOINTS`
 */
extern const std::array<robot::types::motorid_t, 2> IK_MOTORS;

/**
 * Map from motor ids to min and max joint limits in millidegrees
 */
constexpr frozen::unordered_map<robot::types::motorid_t, std::pair<int, int>, IK_MOTORS.size()>
	JOINT_LIMITS{{robot::types::motorid_t::shoulder, {18200, 152500}},
				 {robot::types::motorid_t::elbow, {-169100, 0}}};

/**
 * Map from motor ids to segment length in meters
 */
constexpr frozen::unordered_map<robot::types::motorid_t, double, IK_MOTORS.size()>
	SEGMENT_LENGTHS{{robot::types::motorid_t::shoulder, 0.394},
					{robot::types::motorid_t::elbow, 0.749}};
} // namespace arm

namespace autonomous {
extern const double THETA_KP;
extern const double DRIVE_VEL;
extern const double SLOW_DRIVE_THRESHOLD;
extern const double DONE_THRESHOLD;
// Duration long enough to confirm we are there, not so long that time is wasted
extern const util::dseconds CLOSE_TO_TARGET_DUR_VAL;
} // namespace autonomous

extern const double CONTROL_HZ;

} // namespace Constants
