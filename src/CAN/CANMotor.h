#pragma once

#include "CANUtils.h"
#include "../world_interface/data.h"

#include <chrono>

namespace can::motor {

void initMotor(deviceserial_t serial);

void initMotor(deviceserial_t serial, bool invertEncoder, bool zeroEncoder,
			   int32_t pulsesPerJointRev, std::chrono::milliseconds telemetryPeriod);

void initMotor(deviceserial_t serial, bool invertEncoder, bool zeroEncoder,
			   int32_t pulsesPerJointRev, std::chrono::milliseconds telemetryPeriod,
			   int32_t kP, int32_t kI, int32_t kD);

void setMotorMode(deviceserial_t serial, motormode_t mode);

void setMotorPower(deviceserial_t serial, double power);

void setMotorPower(deviceserial_t serial, int16_t power);

void setMotorPIDTarget(deviceserial_t serial, int32_t target);

robot::types::DataPoint<int32_t> getMotorPosition(deviceserial_t serial);

void pullMotorPosition(deviceserial_t serial);
} // namespace can::motor