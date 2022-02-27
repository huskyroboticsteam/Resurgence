#pragma once

#include "../world_interface/data.h"
#include "CAN.h"

extern "C" {
#include "../HindsightCAN/CANPacket.h"
}

namespace can {
devicegroup_t getDeviceGroup(const CANPacket& packet);

devicegroup_t getDeviceGroup(uint8_t groupCode);

deviceserial_t getDeviceSerial(const CANPacket& packet);

deviceid_t getDeviceGroupAndSerial(const CANPacket& packet);

deviceserial_t getMotorSerial(robot::types::motorid_t motor);
} // namespace can
