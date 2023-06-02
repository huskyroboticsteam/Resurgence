#pragma once

#include "kinematics/DiffDriveKinematics.h"
#include "world_interface/data.h"

#include <array>
#include <chrono>
#include <cmath>
#include <string>

#include <frozen/unordered_map.h>

using namespace kinematics;

namespace Constants {
// TODO: make sure these are still accurate with the new arm.
constexpr double SHOULDER_LENGTH = 0.6; // placeholder(m)
constexpr double ELBOW_LENGTH = 0.7;	// placeholder(m)

/**
   Number of millidegrees per degree
 */
constexpr float MDEG_PER_DEG = 1000;

// TODO: tune these drive constants
constexpr double ROBOT_LENGTH = 1.0;
/**
   Distance between left and right wheels.
*/
constexpr double WHEEL_BASE = 2. / 3.;
/**
  Effective distance between wheels. Tweaked so that actual rover angular rate roughly matches
  the commanded angular rate.
 */
constexpr double EFF_WHEEL_BASE = 1.40;		 // tweaked to match 2-wheel kinematic model
constexpr double WHEEL_RADIUS = 0.15;		 // eyeballed
constexpr double PWM_PER_RAD_PER_SEC = 5000; // eyeballed
constexpr double MAX_DRIVE_PWM = 25000; // can be up to 2^15 - 1 (32767) but we have limited it
										// because driving at full speed draws a massive amount
										// of current that's too much for our power supply (and
										// the rover's full speed is somewhat hard to control
										// while it's driving)
/**
   @brief Maximum tangential velocity for the rover's wheels.

   If the rover is driving straight and not turning, this is the maximum forward velocity
   (i.e. `dx` in setCmdVel()) of the rover.
 */
constexpr double MAX_WHEEL_VEL = WHEEL_RADIUS * MAX_DRIVE_PWM / PWM_PER_RAD_PER_SEC;
/**
   @brief Maximum angular velocity (i.e. `dtheta` in setCmdVel()) of the rover.

   Computed assuming that the left wheel is going at full speed backwards while the right wheel
   is going at full speed forwards.
 */
const double MAX_DTHETA = kinematics::DiffDriveKinematics(EFF_WHEEL_BASE)
							  .wheelVelToRobotVel(-MAX_WHEEL_VEL, MAX_WHEEL_VEL)(2);

// Joint limits
// TODO: make sure these are still accurate with the new arm.
constexpr double ARM_BASE_MIN = -M_PI / 2;
constexpr double ARM_BASE_MAX = M_PI / 2;
// constexpr double SHOULDER_MIN = M_PI / 2; // TODO mechanical problem with the moon gear.
//                                             Use this value during actual rover operation.
constexpr double SHOULDER_MIN = 0.0;
constexpr double SHOULDER_MAX = M_PI * 5. / 6.; // Can't quite lie flat
constexpr double ELBOW_MIN = 0.0;
constexpr double ELBOW_MAX = M_PI * 29. / 30.; // I think this should prevent self-collisions

// TODO: We need to recalibrate the camera, since we replaced it with a different one.
// TODO: rename cameras (in MC as well) as appropriate
constexpr const char* AR_CAMERA_CONFIG_PATH = "../camera-config/MastCameraCalibration.yml";
const robot::types::CameraID AR_CAMERA_ID = "front"; // TODO: replace with real camera name

constexpr const char* FOREARM_CAMERA_CONFIG_PATH =
	"../camera-config/ForearmCameraCalibration.yml";
const robot::types::CameraID FOREARM_CAMERA_ID = "rear";

constexpr const char* HAND_CAMERA_CONFIG_PATH = "../camera-config/HandCameraCalibration.yml";
const robot::types::CameraID HAND_CAMERA_ID = "upperArm";

/**
   @deprecated No need for this constant once we fully switch over the Mission Control PlanViz
 */
constexpr uint16_t PLANVIZ_SERVER_PORT = 9002;
constexpr uint16_t WS_SERVER_PORT = 3001;

/**
   WebSocket server endpoint for the mission control protocol.
 */
constexpr const char* MC_PROTOCOL_NAME = "/mission-control";
/**
   WebSocket server endpoint for the simulator protocol.
 */
constexpr const char* SIM_PROTOCOL_NAME = "/simulator";
/**
   WebSocket server endpoint for the DGPS protocol.
 */
constexpr const char* DGPS_PROTOCOL_NAME = "/dgps";

constexpr std::chrono::milliseconds JOINT_POWER_REPEAT_PERIOD(333);
constexpr std::chrono::milliseconds ARM_IK_UPDATE_PERIOD(100);

namespace Nav {
// Distance (m) we could have traveled forward in the time it takes to turn 1 radian
constexpr double RADIAN_COST = EFF_WHEEL_BASE / 2.0;
// Planner stays this far away from obstacles (m)
constexpr double SAFE_RADIUS = Constants::ROBOT_LENGTH * 1.3;
constexpr int MAX_ITERS = 3000; // Max number of nodes expanded during A* search
constexpr double PLAN_RESOLUTION = Constants::ROBOT_LENGTH; // m
constexpr double SEARCH_RADIUS_INCREMENT = Constants::ROBOT_LENGTH * 3;
constexpr double GPS_WAYPOINT_RADIUS = Constants::ROBOT_LENGTH * 1.5;
constexpr double LANDMARK_WAYPOINT_RADIUS = Constants::ROBOT_LENGTH * 1.3;
constexpr double EPS = 2.0; // heuristic weight for weighted A*
} // namespace Nav

// Lidar
namespace Lidar {
const std::string RP_PATH = "/dev/ttyUSB0";
constexpr double MM_PER_M = 1000;
constexpr uint32_t RPLIDAR_A1_BAUDRATE = 115200;
constexpr uint32_t RPLIDAR_S1_BAUDRATE = 256000;
} // namespace Lidar

// Video encoding
namespace video {
constexpr int H264_RF_CONSTANT = 40;
} // namespace video

// Arm inverse kinematics
namespace arm {

constexpr double IK_SOLVER_THRESH = 0.001;

constexpr int IK_SOLVER_MAX_ITER = 50;

/**
 * The motors used in IK. The ordering in this array is the canonical ordering of these motors
 * for IK purposes.
 */
constexpr std::array<robot::types::motorid_t, 2> IK_MOTORS = {
	robot::types::motorid_t::shoulder, robot::types::motorid_t::elbow};

/**
 * Map from motor ids to min and max joint limits
 */
constexpr frozen::unordered_map<robot::types::motorid_t, std::pair<int, int>, IK_MOTORS.size()>
	JOINT_LIMITS{{robot::types::motorid_t::shoulder, {18200, 152500}},
				 {robot::types::motorid_t::elbow, {-169100, 0}}};

/**
 * Map from motor ids to segment length
 */
constexpr frozen::unordered_map<robot::types::motorid_t, double, IK_MOTORS.size()>
	SEGMENT_LENGTHS{{robot::types::motorid_t::shoulder, 0.3848608},
					{robot::types::motorid_t::elbow, 0.461264}};
} // namespace arm

constexpr double CONTROL_HZ = 10.0;
} // namespace Constants
