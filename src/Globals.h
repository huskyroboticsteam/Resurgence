#pragma once

#include "CommandLineOptions.h"
#include "Networking/Network.h"
#include "Networking/websocket/WebSocketServer.h"

#include <vector>

enum class RoverState {
	RemoteControl,
	DrivingToWaypoint,
	LookingForTennisball,
};

// NOTE(sasha): To keep linker happy, declare globals with "extern" here and then
//              provide variable definition in Globals.cpp
namespace Globals {
extern CommandLineOptions opts;
extern RoverState curr_state;
extern websocket::SingleClientWSServer websocketServer;
extern bool E_STOP;
extern bool AUTONOMOUS;
} // namespace Globals
