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

// NOTE(sasha): To keep linker happy, declare globals with "extern" here and then
//              provide variable definition in Globals.cpp
namespace Globals {
/**
   @deprecated
 */
extern RoverState curr_state;
extern net::websocket::SingleClientWSServer websocketServer;
extern std::atomic<bool> AUTONOMOUS;
extern robot::types::mountedperipheral_t mountedPeripheral;
extern control::PlanarArmController<2> planarArmController;
extern control::SwerveController swerveController;
extern std::atomic<bool> armIKEnabled;
} // namespace Globals
