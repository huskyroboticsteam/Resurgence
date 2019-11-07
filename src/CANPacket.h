#pragma once

struct CANPacket
{
    uint16_t id;
    uint8_t dlc;
    uint8_t data[8];
};

uint16_t ConstructCANID(bool priority, uint8_t devGroup, uint8_t devSerial);
CANPacket ConstructCANPacket(uint16_t id, uint8_t dlc, uint16_t data[8]);

