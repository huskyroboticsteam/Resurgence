#pragma once

namespace net::mc {

// TODO: possibly use frozen::string for this so we don't have to use raw char ptrs
// TODO: MOTOR_POS_REQ, MOTOR_STATUS_REP are unimplemented, do we want to remove or implement?
// request keys
constexpr const char* EMERGENCY_STOP_REQ_TYPE = "emergencyStopRequest";
constexpr const char* OPERATION_MODE_REQ_TYPE = "operationModeRequest";
constexpr const char* DRIVE_MODE_REQ_TYPE = "swerveDriveModeRequest";
constexpr const char* DRIVE_REQ_TYPE = "driveRequest";
constexpr const char* DRIVE_TANK_REQ_TYPE = "tankDriveRequest";
constexpr const char* DRIVE_TURN_IN_PLACE_REQ_TYPE = "turnInPlaceDriveRequest";
constexpr const char* DRIVE_CRAB_REQ_TYPE = "crabDriveRequest";
constexpr const char* ARM_IK_ENABLED_TYPE = "requestArmIKEnabled";
constexpr const char* JOINT_POWER_REQ_TYPE = "jointPowerRequest";
constexpr const char* MOTOR_POSITION_REQ_TYPE = "motorPositionRequest";
constexpr const char* JOINT_POSITION_REQ_TYPE = "jointPositionRequest";
constexpr const char* CAMERA_STREAM_OPEN_REQ_TYPE = "cameraStreamOpenRequest";
constexpr const char* CAMERA_STREAM_CLOSE_REQ_TYPE = "cameraStreamCloseRequest";
constexpr const char* CAMERA_FRAME_REQ_TYPE = "cameraFrameRequest";
constexpr const char* WAYPOINT_NAV_REQ_TYPE = "waypointNavRequest";

// report keys
constexpr const char* MOTOR_STATUS_REP_TYPE = "motorStatusReport";
constexpr const char* CAMERA_STREAM_REP_TYPE = "cameraStreamReport";
constexpr const char* CAMERA_FRAME_REP_TYPE = "cameraFrameReport";
constexpr const char* MOUNTED_PERIPHERAL_REP_TYPE = "mountedPeripheralReport";
constexpr const char* JOINT_POSITION_REP_TYPE = "jointPositionReport";
constexpr const char* ROVER_POS_REP_TYPE = "roverPositionReport";
constexpr const char* ARM_IK_ENABLED_REP_TYPE = "armIKEnabledReport";

} // namespace net::mc
