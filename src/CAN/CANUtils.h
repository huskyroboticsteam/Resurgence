#pragma once

#include "CAN.h"

extern "C" {
#include "../HindsightCAN/CANPacket.h"
}

#include <chrono>

namespace can {
devicegroup_t getDeviceGroup(const CANPacket& packet);

devicegroup_t getDeviceGroup(uint8_t groupCode);

uint8_t getDeviceGroupCode(devicegroup_t group);

deviceserial_t getDeviceSerial(const CANPacket& packet);

deviceid_t getDeviceGroupAndSerial(const CANPacket& packet);

// TODO: implement this and throw away the candevice stuff

void initMotor(deviceserial_t serial);

void initMotor(deviceserial_t serial, bool invertEncoder, bool zeroEncoder,
			   int32_t pulsesPerJointRev, std::chrono::milliseconds telemetryPeriod);

void initMotor(deviceserial_t serial, bool invertEncoder, bool zeroEncoder,
			   int32_t pulsesPerJointRev, std::chrono::milliseconds telemetryPeriod,
			   int32_t kP, int32_t kI, int32_t kD);
} // namespace can
