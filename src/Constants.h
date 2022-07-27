#pragma once

#include "kinematics/DiffDriveKinematics.h"
#include "world_interface/data.h"

#include <cmath>
#include <string>

namespace Constants {
/**
   @deprecated No need for this constant once we fully switch to the new network protocol
 */
constexpr size_t PACKET_PAYLOAD_SIZE = 8;
// TODO: make sure these are still accurate with the new arm.
constexpr double SHOULDER_LENGTH = 0.6; // placeholder(m)
constexpr double ELBOW_LENGTH = 0.7;	// placeholder(m)

/**
   Number of millidegrees per degree
 */
constexpr float MDEG_PER_DEG = 1000;

constexpr std::array<uint32_t, 6> arm_PPJRs = {
	17 * 1000, // base, estimate

	20 * 1000, // shoulder, estimate
	36 * 1000, // elbow, rough estimate

	360 * 1000, // forearm, unmeasured
	360 * 1000, // diff_left, unmeasured
	360 * 1000	// diff_right, unmeasured
};

// So far only the base, shoulder, elbow have been tuned
//
// base, shoulder, elbow, forearm, diff_left, diff_right
constexpr std::array<int32_t, 6> arm_Ps = {1000, 100, 500, 0, 0, 0};
constexpr std::array<int32_t, 6> arm_Is = {50, 0, 50, 0, 0, 0};
constexpr std::array<int32_t, 6> arm_Ds = {10000, 1000, 10000, 0, 0, 0};
constexpr std::array<uint8_t, 6> arm_encoder_signs = {0, 0, 1, 0, 0, 0};

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
const double MAX_DTHETA =
	DiffDriveKinematics(EFF_WHEEL_BASE).wheelVelToRobotVel(-MAX_WHEEL_VEL, MAX_WHEEL_VEL)(2);

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
constexpr const char* AR_CAMERA_CONFIG_PATH = "../camera-config/MastCameraCalibration.yml";
const robot::types::CameraID AR_CAMERA_ID = "front"; // TODO: replace with real camera name

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

namespace Nav {
// Distance (m) we could have traveled forward in the time it takes to turn 1 radian
const double RADIAN_COST = EFF_WHEEL_BASE / 2.0;
// Planner stays this far away from obstacles (m)
const double SAFE_RADIUS = Constants::ROBOT_LENGTH * 1.3;
const int MAX_ITERS = 3000; // Max number of nodes expanded during A* search
const double PLAN_RESOLUTION = Constants::ROBOT_LENGTH; // m
const double SEARCH_RADIUS_INCREMENT = Constants::ROBOT_LENGTH * 3;
const double GPS_WAYPOINT_RADIUS = Constants::ROBOT_LENGTH * 1.5;
const double LANDMARK_WAYPOINT_RADIUS = Constants::ROBOT_LENGTH * 1.3;
const double EPS = 2.0; // heuristic weight for weighted A*
} // namespace Nav

// Lidar
namespace Lidar {
const std::string RP_PATH = "/dev/ttyUSB0";
constexpr double MM_PER_M = 1000;
constexpr uint32_t RPLIDAR_A1_BAUDRATE = 115200;
constexpr uint32_t RPLIDAR_S1_BAUDRATE = 256000;
}

constexpr double CONTROL_HZ = 10.0;
} // namespace Constants
