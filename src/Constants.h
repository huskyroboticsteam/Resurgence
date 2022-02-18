#pragma once

#include <cmath>
#include <cstddef>
#include <string>
#include "world_interface/data.h"
#include "kinematics/DiffDriveKinematics.h"

namespace Constants {
/**
   @deprecated No need for this constant once we fully switch to the new network protocol
 */
constexpr size_t PACKET_PAYLOAD_SIZE = 8;
// TODO: make sure these are still accurate with the new arm.
constexpr double SHOULDER_LENGTH = 0.6; // placeholder(m)
constexpr double ELBOW_LENGTH = 0.7;	// placeholder(m)

// TODO: tune these drive constants
constexpr double ROBOT_LENGTH = 1.0;
constexpr double WHEEL_BASE = 2. / 3.;
constexpr double EFF_WHEEL_BASE = 1.40; // tweaked to match 2-wheel kinematic model
constexpr double WHEEL_RADIUS = 0.15; // eyeballed
constexpr double PWM_PER_RAD_PER_SEC = 5000; // eyeballed
constexpr double MAX_DRIVE_PWM = 20000;
/**
   @brief Maximum tangential velocity for the rover's wheels.

   If the rover is driving straight and not turning, this is the maximum forward velocity
   (i.e. `dx` in setCmdVel()) of the rover.
 */
constexpr double MAX_WHEEL_VEL = WHEEL_RADIUS * MAX_DRIVE_PWM / PWM_PER_RAD_PER_SEC;
/**
   Kinematic model for the rover, based on the wheel base width Constants::EFF_WHEEL_BASE
 */
const DiffDriveKinematics kinematics(EFF_WHEEL_BASE);
/**
   @brief Maximum angular velocity (i.e. `dtheta` in setCmdVel()) of the rover.

   Computed assuming that the left wheel is going at full speed backwards while the right wheel
   is going at full speed forwards.
 */
const double MAX_DTHETA = kinematics.wheelVelToRobotVel(-MAX_WHEEL_VEL, MAX_WHEEL_VEL)(2);

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

constexpr const char* AR_CAMERA_CONFIG_PATH = "../camera-config/MastCameraCalibration.yml";
const CameraID AR_CAMERA_ID = "AR_CAMERA"; // TODO: replace with real camera name

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
const double RADIAN_COST = EFF_WHEEL_BASE / 2.0; // Distance (m) we could have traveled forward in the time it takes to turn 1 radian
const double SAFE_RADIUS = Constants::ROBOT_LENGTH * 1.3; // Planner stays this far away from obstacles (m)
const int MAX_ITERS = 3000; // Max number of nodes expanded during A* search
const double PLAN_RESOLUTION = Constants::ROBOT_LENGTH; // m
const double SEARCH_RADIUS_INCREMENT = Constants::ROBOT_LENGTH*3;
const double GPS_WAYPOINT_RADIUS = Constants::ROBOT_LENGTH * 1.5;
const double LANDMARK_WAYPOINT_RADIUS = Constants::ROBOT_LENGTH * 1.3;
const double EPS = 2.0; // heuristic weight for weighted A*
}
} // namespace Constants
