#pragma once

#include "world_interface/data.h"

#include <cmath>
#include <string>

namespace Constants {
constexpr size_t PACKET_PAYLOAD_SIZE = 8;
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
constexpr double WHEEL_BASE = 2. / 3.;
constexpr double EFF_WHEEL_BASE = 1.40;		 // tweaked to match 2-wheel kinematic model
constexpr double WHEEL_RADIUS = 0.15;		 // eyeballed
constexpr double PWM_PER_RAD_PER_SEC = 5000; // eyeballed
constexpr double MAX_DRIVE_PWM = 20000;
constexpr double MAX_WHEEL_VEL = WHEEL_RADIUS * MAX_DRIVE_PWM / PWM_PER_RAD_PER_SEC;

// Joint limits
constexpr double ARM_BASE_MIN = -M_PI / 2;
constexpr double ARM_BASE_MAX = M_PI / 2;
// constexpr double SHOULDER_MIN = M_PI / 2; // TODO mechanical problem with the moon gear.
//                                             Use this value during actual rover operation.
constexpr double SHOULDER_MIN = 0.0;
constexpr double SHOULDER_MAX = M_PI * 5. / 6.; // Can't quite lie flat
constexpr double ELBOW_MIN = 0.0;
constexpr double ELBOW_MAX = M_PI * 29. / 30.; // I think this should prevent self-collisions

const std::string AR_CAMERA_CONFIG_PATH = "../camera-config/MastCameraCalibration.yml";
const robot::types::CameraID AR_CAMERA_ID = "AR_CAMERA"; // TODO: replace with real camera name

constexpr uint16_t PLANVIZ_SERVER_PORT = 9002;
constexpr uint16_t WS_SERVER_PORT = 3001;

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
} // namespace Constants
