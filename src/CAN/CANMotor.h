#pragma once

#include "../world_interface/data.h"
#include "CAN.h"
#include "CANUtils.h"

#include <bitset>
#include <chrono>

/**
 * @namespace can::motor
 * @brief Utilities for interacting with motor boards over a CAN interface.
 *
 * These motor boards can either be AVR or PSoC boards.
 */
namespace can::motor {

/** @brief The possible motor modes. */
enum class motormode_t { pwm = MOTOR_UNIT_MODE_PWM, pid = MOTOR_UNIT_MODE_PID };

// TODO: write documentation for the following methods

void emergencyStopMotors();

void initMotor(deviceserial_t serial);

void initMotor(deviceserial_t serial, bool invertEncoder, bool zeroEncoder,
			   int32_t pulsesPerJointRev, std::chrono::milliseconds telemetryPeriod);

void initMotor(deviceserial_t serial, bool invertEncoder, bool zeroEncoder,
			   int32_t pulsesPerJointRev, std::chrono::milliseconds telemetryPeriod,
			   int32_t kP, int32_t kI, int32_t kD);

void setMotorPIDConstants(deviceserial_t serial, int32_t kP, int32_t kI, int32_t kD);

void setMotorMode(deviceserial_t serial, motormode_t mode);

void setMotorPower(deviceserial_t serial, double power);

void setMotorPower(deviceserial_t serial, int16_t power);

void setMotorPIDTarget(deviceserial_t serial, int32_t target);

robot::types::DataPoint<int32_t> getMotorPosition(deviceserial_t serial);

void pullMotorPosition(deviceserial_t serial);

callbackid_t addLimitSwitchCallback(
	deviceserial_t serial,
	const std::function<void(deviceserial_t serial,
					   robot::types::DataPoint<robot::types::LimitSwitchData> limitSwitchData)>&
		callback);

void removeLimitSwitchCallback(callbackid_t id);
} // namespace can::motor
