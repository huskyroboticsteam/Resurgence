#pragma once

#include <stdint.h>


struct CANPacket
{
    uint16_t id;
    uint8_t dlc;
    uint8_t data[8];
};

uint16_t ConstructCANID(uint8_t priority, uint8_t devGroup, uint8_t devSerial);
struct CANPacket ConstructCANPacket(uint16_t id, uint8_t dlc, char* data);
uint8_t ParseDataSenderDevice(char* data);
uint8_t ParseDataSenderSerial(char* data);
uint8_t ParseDataPayloadType(char* data);
