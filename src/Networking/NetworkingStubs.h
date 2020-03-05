
#pragma once

extern "C"
{
    #include "../HindsightCAN/CANPacket.h"
}
#include <string>

// For testing purposes. The implementations of sendCANPacket and sendBaseStationPacket in
// NetworkingStubs.cpp will just save their arguments into an array
// which can be retrieved using popCANPacket and popBaseStationPacket

int numCANPackets();
int numBaseStationPackets();
CANPacket popCANPacket();
std::string popBaseStationPacket();
