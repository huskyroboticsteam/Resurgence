#pragma once

namespace net::mc {

// TODO: possibly use frozen::string for this so we don't have to use raw char ptrs
// request keys
constexpr const char* EMERGENCY_STOP_REQ_TYPE = "emergencyStopRequest";
constexpr const char* OPERATION_MODE_REQ_TYPE = "operationModeRequest";
constexpr const char* DRIVE_REQ_TYPE = "driveRequest";
constexpr const char* ARM_IK_ENABLED_TYPE = "requestArmIKEnabled";
constexpr const char* MOTOR_POWER_REQ_TYPE = "motorPowerRequest";
constexpr const char* JOINT_POWER_REQ_TYPE = "jointPowerRequest";
constexpr const char* MOTOR_POSITION_REQ_TYPE = "motorPositionRequest";
constexpr const char* JOINT_POSITION_REQ_TYPE = "jointPositionRequest";
constexpr const char* CAMERA_STREAM_OPEN_REQ_TYPE = "cameraStreamOpenRequest";
constexpr const char* CAMERA_STREAM_CLOSE_REQ_TYPE = "cameraStreamCloseRequest";

// report keys
constexpr const char* MOTOR_STATUS_REP_TYPE = "motorStatusReport";
constexpr const char* CAMERA_STREAM_REP_TYPE = "cameraStreamReport";
constexpr const char* LIDAR_REP_TYPE = "lidarReport";
constexpr const char* MOUNTED_PERIPHERAL_REP_TYPE = "mountedPeripheralReport";
constexpr const char* JOINT_POSITION_REP_TYPE = "jointPositionReport";
constexpr const char* ROVER_POS_REP_TYPE = "roverPositionReport";
constexpr const char* ARM_IK_ENABLED_REP_TYPE = "armIKEnabledReport";

} // namespace net::mc
