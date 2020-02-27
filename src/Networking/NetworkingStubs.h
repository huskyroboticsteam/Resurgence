
#pragma once

extern "C"
{
    #include "../HindsightCAN/CANPacket.h"
}
#include <string>

// For testing purposes. The implementations of SendCANPacket and sendBaseStationPacket in
// NetworkingStubs.cpp will just save their arguments into an array
// which can be retrieved using popCANPacket and popBaseStationPacket

CANPacket popCANPacket();
std::string popBaseStationPacket();
