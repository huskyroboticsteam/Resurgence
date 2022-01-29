#include "Globals.h"

#include "CommandLineOptions.h"

#include <vector>

#include <nlohmann/json.hpp>

namespace Globals {
CommandLineOptions opts;
RoverState curr_state;
nlohmann::json status_data;
nlohmann::json motor_status;
bool E_STOP = false;
bool AUTONOMOUS = false;
} // namespace Globals
