#pragma once

#include "CAN.h"

extern "C" {
#include "../HindsightCAN/CANPacket.h"
}

namespace can {
devicegroup_t getDeviceGroup(const CANPacket& packet);

devicegroup_t getDeviceGroup(uint8_t groupCode);

uint8_t getDeviceGroupCode(devicegroup_t group);

deviceserial_t getDeviceSerial(const CANPacket& packet);

deviceid_t getDeviceGroupAndSerial(const CANPacket& packet);
} // namespace can
