#pragma once

extern "C" {
#include "../HindsightCAN/CANPacket.h"
}

#include <chrono>
#include <utility>

namespace can {
enum class devicegroup_t {
	broadcast,
	reserved,
	master,
	power,
	motor,
	telemetry,
	gpio,
	science
};

using deviceserial_t = uint8_t;
using telemtype_t = uint8_t; // TODO: convert to enum
using telemetry_t = int32_t;

using deviceid_t = std::pair<devicegroup_t, deviceserial_t>;

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
