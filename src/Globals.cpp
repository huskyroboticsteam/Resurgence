#include "Globals.h"

#include "CommandLineOptions.h"
#include "Constants.h"

#include <vector>

#include <nlohmann/json.hpp>

namespace Globals {
CommandLineOptions opts;
RoverState curr_state;
websocket::SingleClientWSServer websocketServer("DefaultServer", Constants::WS_SERVER_PORT);
bool E_STOP = false;
bool AUTONOMOUS = false;
} // namespace Globals
