#pragma once

#include <cmath>
#include <cstddef>
#include <string>
#include "world_interface/data.h"

namespace Constants {
constexpr size_t PACKET_PAYLOAD_SIZE = 8;
constexpr double SHOULDER_LENGTH = 0.6; // placeholder(m)
constexpr double ELBOW_LENGTH = 0.7;	// placeholder(m)

// TODO: tune these drive constants
constexpr double WHEEL_BASE = 2. / 3.;
constexpr double EFF_WHEEL_BASE = 1.40; // tweaked to match 2-wheel kinematic model
constexpr double WHEEL_RADIUS = 0.15; // eyeballed
constexpr double PWM_PER_RAD_PER_SEC = 5000; // eyeballed
constexpr double MAX_DRIVE_PWM = 20000;

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
const CameraID AR_CAMERA_ID = "AR_CAMERA"; // TODO: replace with real camera name

constexpr uint16_t PLANVIZ_SERVER_PORT = 9002;
constexpr uint16_t WS_SERVER_PORT = 3001;
} // namespace Constants
