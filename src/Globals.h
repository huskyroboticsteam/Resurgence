#pragma once

#include "control/PlanarArmController.h"
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

enum class DriveMode {
	Normal,
	TurnInPlace,
	Crab,
};

static const std::map<DriveMode, std::string> driveModeStrings = {
	{DriveMode::Normal, "Normal"},
	{DriveMode::TurnInPlace, "TurnInPlace"},
	{DriveMode::Crab, "Crab"}};

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
extern std::atomic<bool> armIKEnabled;
} // namespace Globals
