#pragma once

#include "Globals.h"
#include "Constants.h"

#include <cstdint>

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

void InitializeNetwork();
void ParseIncomingNetworkPackets();
void SendOutgoingNetworkPackets();
void TestCANPackets();
