#include "Globals.h"

#include "CommandLineOptions.h"
#include "Constants.h"

#include <vector>
#include <atomic>

namespace Globals {
CommandLineOptions opts;
RoverState curr_state;
net::websocket::SingleClientWSServer websocketServer("DefaultServer", Constants::WS_SERVER_PORT);
std::atomic<bool> E_STOP = false;
std::atomic<bool> AUTONOMOUS = false;
} // namespace Globals
