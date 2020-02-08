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

enum class PacketKind
{
    CAN,
    Network,
};

struct Packet
{
    PacketKind kind;
    int address;
    uint8_t payload[Constants::PACKET_PAYLOAD_SIZE];
};

void InitializeBaseStationSocket();
int recvBaseStationPacket(char *buffer);
void sendBaseStationPacket(const std::string &packet);
