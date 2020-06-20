#pragma once

#include "../Constants.h"
#include "json.hpp"

extern "C"
{
    #include "../HindsightCAN/CANPacket.h"
    #include "../HindsightCAN/CANCommon.h"
}

#include <cstdint>
#include <aio.h>
#include <fcntl.h>

void InitializeBaseStationSocket();
int recvBaseStationPacket(char *buffer);
void sendBaseStationPacket(const std::string &packet);
