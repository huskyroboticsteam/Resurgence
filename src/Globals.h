#pragma once

#include "CommandLineOptions.h"
#include "Networking/Network.h"
#include "Networking/websocket/WebSocketServer.h"

#include <vector>

#include <nlohmann/json.hpp>

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
extern CommandLineOptions opts;
/**
   @deprecated
 */
extern RoverState curr_state;
/**
   @deprecated
 */
extern nlohmann::json status_data;
/**
   @deprecated
 */
extern nlohmann::json motor_status;
extern websocket::SingleClientWSServer websocketServer;
// TODO(evan): make these atomic
extern bool E_STOP;
extern bool AUTONOMOUS;
} // namespace Globals
