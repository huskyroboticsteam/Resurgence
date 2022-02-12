#pragma once

#include "../websocket/WebSocketProtocol.h"
#include "../../world_interface/world_interface.h"
#include <unordered_set>

namespace mc {
using json = nlohmann::json;

websocket::WebSocketProtocol initMissionControlProtocol();
std::unordered_set<CameraID> getOpenCameraStreams();

// request keys
constexpr const char* EMERGENCY_STOP_REQ_TYPE = "emergencyStopRequest";
constexpr const char* OPERATION_MODE_REQ_TYPE = "operationModeRequest";
constexpr const char* DRIVE_REQ_TYPE = "driveRequest";
constexpr const char* MOTOR_POWER_REQ_TYPE = "motorPowerRequest";
constexpr const char* CAMERA_STREAM_OPEN_REQ_TYPE = "cameraStreamOpenRequest";
constexpr const char* CAMERA_STREAM_CLOSE_REQ_TYPE = "cameraStreamCloseRequest";

// report keys
constexpr const char* MOTOR_STATUS_REP_TYPE = "motorStatusReport";
constexpr const char* CAMERA_STREAM_REP_TYPE = "cameraStreamReport";
constexpr const char* LIDAR_REP_TYPE = "lidarReport";
};
