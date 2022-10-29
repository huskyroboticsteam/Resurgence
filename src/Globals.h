#pragma once

#include "network/websocket/WebSocketServer.h"

#include <vector>
#include <atomic>

/**
   @deprecated
 */
enum class RoverState {
	RemoteControl,
	DrivingToWaypoint,
	LookingForTennisball,
};

enum class mountedperipheral_t {
	none,
	arm,
	scienceStation,
	lidar
};

// NOTE(sasha): To keep linker happy, declare globals with "extern" here and then
//              provide variable definition in Globals.cpp
namespace Globals {
/**
   @deprecated
 */
extern RoverState curr_state;
extern net::websocket::SingleClientWSServer websocketServer;
extern std::atomic<bool> E_STOP;
extern std::atomic<bool> AUTONOMOUS;
extern mountedperipheral_t mountedPeripheral;
} // namespace Globals
