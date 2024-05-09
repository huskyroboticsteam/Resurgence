#pragma once

#include "control/PlanarArmController.h"
#include "control/SwerveController.h"
#include "kinematics/ArmKinematics.h"
#include "network/websocket/WebSocketServer.h"
#include "world_interface/data.h"

#include <atomic>
#include <vector>

/**
   @deprecated
 */
enum class RoverState {
	RemoteControl,
	DrivingToWaypoint,
	LookingForTennisball,
};

namespace control {
enum class DriveMode;
class SwerveController;
} // namespace control

using control::DriveMode;

// NOTE(sasha): To keep linker happy, declare globals with "extern" here and then
//              provide variable definition in Globals.cpp
namespace Globals {
/**
   @deprecated
 */
// The boolean here is whether or not wheel rotation check should be ignored when doing
// swerve-based driving
extern std::pair<DriveMode, bool> driveMode;
extern RoverState curr_state;
extern net::websocket::SingleClientWSServer websocketServer;
extern std::atomic<bool> E_STOP;
extern std::atomic<bool> AUTONOMOUS;
extern robot::types::mountedperipheral_t mountedPeripheral;
extern control::PlanarArmController<2> planarArmController;
extern control::SwerveController swerveController;
extern std::atomic<bool> armIKEnabled;
} // namespace Globals
