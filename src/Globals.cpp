#include "CommandLineOptions.h"
#include "Globals.h"
#include "Networking/json.hpp"

#include <vector>

CommandLineOptions Globals::opts;
RoverState Globals::curr_state;
nlohmann::json Globals::status_data;
nlohmann::json Globals::motor_status;
