#pragma once

#include "../Constants.h"
#include <nlohmann/json.hpp>

extern "C" {
#include "../HindsightCAN/CANCommon.h"
#include "../HindsightCAN/CANPacket.h"
}

#include <aio.h>
#include <cstdint>
#include <fcntl.h>

bool InitializeBaseStationSocket();
int recvBaseStationPacket(char* buffer);
void sendBaseStationPacket(const std::string& packet);
