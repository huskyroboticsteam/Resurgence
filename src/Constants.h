#pragma once

#include "utils/time.h"
#include "world_interface/data.h"

#include <array>
#include <chrono>
#include <cmath>
#include <string>
#include <unordered_set>

#include <frozen/unordered_map.h>

using robot::types::CameraID;
using robot::types::jointid_t;
using robot::types::motorid_t;

/**
   @namespace Constants
*/
namespace Constants {

/**
   @namespace Constants::Arm
*/
namespace Arm {
/**
   @brief Length of the Shoulder link, in meters
*/
extern const double SHOULDER_LENGTH;
/**
   @brief Length of the Elbow link, in meters
*/
extern const double ELBOW_LENGTH;

/**
   @brief Percentage of fully extended overall arm length to limit arm extension within.
 */
extern const double SAFETY_FACTOR;

/**
   @brief Maximum commanded end-effector velocity, in m/s
 */
extern const double MAX_EE_VEL;
extern const double IK_SOLVER_THRESH;
extern const int IK_SOLVER_MAX_ITER;

/**
   @brief The joints corresponding to the motors used for IK in the arm.
   
   The ordering in this array is the canonical ordering of these joints for IK purposes.
 */
extern const std::array<jointid_t, 2> IK_MOTOR_JOINTS;

/**
   @brief The motors used in IK.
   
   The i-th element in this array corresponds to the joint in the i-th element of `IK_MOTOR_JOINTS`
 */
extern const std::array<motorid_t, 2> IK_MOTORS;

/**
   @brief Map from motor ids to min and max joint limits in millidegrees
 */
constexpr frozen::unordered_map<motorid_t, std::pair<int, int>, IK_MOTORS.size()> JOINT_LIMITS{
	{motorid_t::shoulder, {18200, 152500}}, {motorid_t::elbow, {-169100, 0}}};

/**
   @brief Map from motor ids to segment length in meters
 */
constexpr frozen::unordered_map<motorid_t, double, IK_MOTORS.size()> SEGMENT_LENGTHS{
	{motorid_t::shoulder, 0.3848608}, {motorid_t::elbow, 0.461264}};

} // namespace Arm


/**
   @namespace Constants::Drive
*/
namespace Drive {
extern const double ROBOT_LENGTH;
extern const double ROBOT_WIDTH;

/**
   @brief Distance between left and right wheels.
*/
extern const double WHEEL_BASE;
/**
  @brief Effective distance between wheels.
  
  Tweaked so that actual rover angular rate roughly matches
  the commanded angular rate, based on 2-wheel kinematic model.
 */
extern const double EFF_WHEEL_BASE;
extern const double WHEEL_RADIUS;
extern const double PWM_PER_RAD_PER_SEC;
/**
   Can be up to 2^15 - 1 (32767), but we have limited it because
   driving at full speed draws a massive amount of current that's
   too much for our power supply (and full speed is hard to control anyway)
*/
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

/**
   Represents the allowable error in millidegrees for steer motors to still process a drive
   request. That is, we make sure all the wheels are close enough to their target rotation
   before actually driving.
*/
extern const double STEER_EPSILON;

} // namespace Drive


/**
   @namespace Constants::Camera
*/
namespace Camera {
extern const CameraID MAST_CAMERA_ID;
extern const CameraID WRIST_CAMERA_ID;
extern const CameraID HAND_CAMERA_ID;

extern const std::unordered_set<robot::types::CameraID> CAMERA_SET;
extern const std::unordered_map<robot::types::CameraID, std::string> CAMERA_CONFIG_PATHS;

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
extern const std::unordered_map<CameraID, int> STREAM_RFS;

} // namespace Camera


/**
   @namespace Constants::Network
*/
namespace Network {
extern const uint16_t WS_SERVER_PORT;

/**
   @brief WebSocket server endpoint for the mission control protocol.
 */
extern const char* MC_PROTOCOL_NAME;
/**
   @brief WebSocket server endpoint for the simulator protocol.
 */
extern const char* SIM_PROTOCOL_NAME;
/**
   @brief Websocket server endpoint for ArduPilot protocol
 */
extern const char* ARDUPILOT_PROTOCOL_NAME;

} // namespace Network


/**
   @namespace Constants::Nav
*/
namespace Nav {
/**
   @brief Distance (m) we could have traveled forward in the time it takes to turn 1 radian
*/
extern const double RADIAN_COST;
/**
   @brief Planner stays this far away from obstacles (m)
*/
extern const double SAFE_RADIUS;
/**
   @brief Max number of nodes expanded during A* search
*/
extern const int MAX_ITERS;
/**
   @brief In meters
*/
extern const double PLAN_RESOLUTION;
extern const double SEARCH_RADIUS_INCREMENT;
extern const double GPS_WAYPOINT_RADIUS;
extern const double LANDMARK_WAYPOINT_RADIUS;
/**
   @brief Heuristic weight for weighted A*
*/
extern const double EPS;

extern const double THETA_KP;
extern const double DRIVE_VEL;
extern const double SLOW_DRIVE_THRESHOLD;
extern const double DONE_THRESHOLD;
/**
   Duration long enough to confirm we are there, not so long that time is wasted
*/
extern const util::dseconds CLOSE_TO_TARGET_DUR_VAL;

} // namespace Nav


/**
 * A map that pairs each of the joints to its corresponding motor.
 * (one-to-one pairs only)
 */
constexpr auto JOINT_MOTOR_MAP = frozen::make_unordered_map<jointid_t, motorid_t>(
	{{jointid_t::armBase, motorid_t::armBase},
	 {jointid_t::shoulder, motorid_t::shoulder},
	 {jointid_t::elbow, motorid_t::elbow},
	 {jointid_t::forearm, motorid_t::forearm},
	 {jointid_t::hand, motorid_t::hand},
	 {jointid_t::drillActuator, motorid_t::drillActuator},
	 {jointid_t::drillMotor, motorid_t::drillMotor}});

extern const double CONTROL_HZ;

extern const std::chrono::milliseconds JOINT_POWER_REPEAT_PERIOD;
extern const std::chrono::milliseconds ARM_IK_UPDATE_PERIOD;

} // namespace Constants
