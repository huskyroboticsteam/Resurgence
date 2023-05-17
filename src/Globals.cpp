#include "Globals.h"
#include "Constants.h"
#include "world_interface/data.h"

#include <vector>
#include <atomic>

namespace Globals {
RoverState curr_state;
net::websocket::SingleClientWSServer websocketServer("DefaultServer", Constants::WS_SERVER_PORT);
std::atomic<bool> E_STOP = false;
std::atomic<bool> AUTONOMOUS = false;
robot::types::mountedperipheral_t mountedPeripheral = robot::types::mountedperipheral_t::none;
std::atomic<bool> armIKEnabled = false;
} // namespace Globals
