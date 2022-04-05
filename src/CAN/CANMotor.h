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

/**
 * @brief The maximum number of limit switches associated with any motor.
 */
constexpr size_t N_LIMIT_SWITCH = 8;

/**
 * @brief A class that represents limit switches from a specific motor.
 */
class LimitSwitchData {
public:
	/**
	 * @brief Construct a new LimitSwitchData object, from the given bits.
	 *
	 * A 1 bit means closed, and 0 means open.
	 * Only the rightmost N_LIMIT_SWITCH bits are used.
	 *
	 * @param data The limit switch data.
	 */
	LimitSwitchData(unsigned long long data);

	/**
	 * @brief Check if the given index is open.
	 *
	 * This method is the opposite of LimitSwitchData::isClosed().
	 *
	 * @param idx The index to check. 0 <= idx < N_LIMIT_SWITCH.
	 * @return bool True iff the switch at the given index is open.
	 */
	bool isOpen(size_t idx);

	/**
	 * @brief Check if the given index is closed.
	 *
	 * This method is the opposite of LimitSwitchData::isOpen().
	 *
	 * @param idx The index to check. 0 <= idx < N_LIMIT_SWITCH.
	 * @return bool True iff the switch at the given index is closed.
	 */
	bool isClosed(size_t idx);

	/**
	 * @brief Check if any index is open.
	 *
	 * @return bool True iff any index is open.
	 */
	bool isAnyOpen();

	/**
	 * @brief Check if any index is closed.
	 *
	 * @return bool True iff any index is closed.
	 */
	bool isAnyClosed();

	/**
	 * @brief Check which indices differ between this data and other.
	 *
	 * This is useful to see which indices have recently changed.
	 *
	 * @param other The data to check against.
	 * @return std::bitset<N_LIMIT_SWITCH> A bitset where an index is 1 if
	 * that this and other differ at that index, and 0 otherwise.
	 */
	std::bitset<N_LIMIT_SWITCH> diff(const LimitSwitchData& other);

private:
	std::bitset<N_LIMIT_SWITCH> data;
};

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
	std::function<void(deviceserial_t serial,
					   robot::types::DataPoint<LimitSwitchData> limitSwitchData)>
		callback);

void removeLimitSwitchCallback(callbackid_t id);
} // namespace can::motor
