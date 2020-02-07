#include "CommandLineOptions.h"
#include "Globals.h"
#include "Network.h"
#include "Networking/json.hpp"

#include <vector>

CommandLineOptions Globals::opts;
RoverState Globals::curr_state;
std::vector<Packet> Globals::incoming_packets;
std::vector<Packet> Globals::outgoing_packets;
nlohmann::json Globals::status_data;
