#include "CommandLineOptions.h"
#include "Globals.h"
#include "Network.h"

#include <vector>

CommandLineOptions Globals::opts;
RoverState Globals::curr_state;
std::vector<Packet> Globals::incoming_packets;
std::vector<Packet> Globals::outgoing_packets;
int Globals::can_fd;
int Globals::net_fd;
