#pragma once

extern "C" {
#include "../HindsightCAN/CANPacket.h"
#include "../HindsightCAN/CANMotorUnit.h"
}

#include <chrono>
#include <utility>

namespace can {
enum class devicegroup_t {
	broadcast = DEVICE_GROUP_BROADCAST,
	reserved = DEVICE_GROUP_RESERVED,
	master = DEVICE_GROUP_MASTER,
	power = DEVICE_GROUP_POWER,
	motor = DEVICE_GROUP_MOTOR_CONTROL,
	telemetry = DEVICE_GROUP_TELEMETRY,
	gpio = DEVICE_GROUP_GPIO_BOARDS,
	science = DEVICE_GROUP_SCIENCE
};

enum class motormode_t {
	pwm = MOTOR_UNIT_MODE_PWM,
	pid = MOTOR_UNIT_MODE_PID
};

using deviceserial_t = uint8_t;
using telemtype_t = uint8_t; // TODO: convert to enum
using telemetry_t = int32_t;

using deviceid_t = std::pair<devicegroup_t, deviceserial_t>;

devicegroup_t getDeviceGroup(const CANPacket& packet);

deviceserial_t getDeviceSerial(const CANPacket& packet);

deviceid_t getDeviceGroupAndSerial(const CANPacket& packet);

// TODO: implement this and throw away the candevice stuff

void initMotor(deviceserial_t serial);

void initMotor(deviceserial_t serial, bool invertEncoder, bool zeroEncoder,
			   int32_t pulsesPerJointRev, std::chrono::milliseconds telemetryPeriod);

void initMotor(deviceserial_t serial, bool invertEncoder, bool zeroEncoder,
			   int32_t pulsesPerJointRev, std::chrono::milliseconds telemetryPeriod,
			   int32_t kP, int32_t kI, int32_t kD);

void setMotorMode(deviceserial_t serial, motormode_t mode);
} // namespace can
