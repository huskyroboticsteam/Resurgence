#pragma once

#include "../world_interface/data.h"
#include "CAN.h"
#include "CANUtils.h"

#include <chrono>
#include <optional>

/**
 * @namespace can::motor
 * @brief Utilities for interacting with motor boards over a CAN interface.
 *
 * These motor boards can either be AVR or PSoC boards.
 */
namespace can::motor {

/** @brief The possible motor modes. */
enum class motormode_t {
	pwm = MOTOR_UNIT_MODE_PWM,
	pid = MOTOR_UNIT_MODE_PID
};

/** @brief The supported motor position sensors. */
struct sensor_t {
	enum {
		encoder = 0,
		potentiometer = 1
	};
};

/**
 * @brief Emergency stop all motors on the CAN bus.
 *
 * This cuts power to all motors and resets them.
 */
void emergencyStopMotors();

/**
 * @brief Initialize a motor.
 *
 * This does not initialize any sensor or controller.
 *
 * @param serial The CAN serial number of the motor to initialize.
 */
void initMotor(deviceserial_t serial);

/**
 * @brief Initialize an encoder attached to the given motor.
 *
 * For potentiometers, use initPotentiometer().
 *
 * @param serial The CAN serial number of the motor board.
 * @param invertEncoder If true, invert the encoder direction. Use this to correct sensor
 * phase.
 * @param zeroEncoder If true, reset the encoder position to zero.
 * @param pulsesPerJointRev The number of encoder pulses per revolution of the physical joint.
 * Measure/calculate this using the gear ratios and encoder specs.
 * @param telemetryPeriod An optional parameter specifying the telemetry period.
 * The telemetry will be fetched at this period automatically. An empty optional disables this
 * behavior, in which case the motor position must be explicitly pulled.
 */
void initEncoder(deviceserial_t serial, bool invertEncoder, bool zeroEncoder,
				 int32_t pulsesPerJointRev,
				 std::optional<std::chrono::milliseconds> telemetryPeriod);

/**
 * @brief Set the limits of the limit switch on a motor board.
 *
 * When the corresponding limit switch is triggered, the encoder value is set to this value.
 * Use this method for motorboard with both encoders and limit switches
 *
 * @param serial The CAN serial number of the motor board.
 * @param lo The joint position in encoder ticks of the low limit switch.
 * @param hi The joint position in encoder ticks of the high limit switch.
 */
void setLimitSwitchLimits(deviceserial_t serial, int32_t lo, int32_t hi);

/**
 * @brief Initialize a potentiometer attached to the given motor.
 *
 * @param serial The CAN serial number of the motor board.
 * @param posLo The joint position that corresponds to @p adcLo
 * @param posHi The joint position that corresponds to @p adcHi
 * @param adcLo The ADC value when the joint is at @p posLo
 * @param adcHi The ADC value when the joint is at @p posHi
 * @param telemetryPeriod An optional parameter specifying the telemetry period.
 * The telemetry will be fetched at this period automatically. An empty optional disables this
 * behavior, in which case the motor position must be explicitly pulled.
 */
void initPotentiometer(deviceserial_t serial, int32_t posLo, int32_t posHi, uint16_t adcLo,
					   uint16_t adcHi,
					   std::optional<std::chrono::milliseconds> telemetryPeriod);

/**
 * @brief Set the PID constants for a motor board.
 *
 * Note that the PID constants are specified in units of 1000, so a 1 is interpreted as a
 * 1000. This is because the controller operates on millidegrees.
 *
 * @param serial The CAN serial number of the motor board.
 * @param kP The P coefficient.
 * @param kI The I coefficient.
 * @param kD The D coefficient.
 */
void setMotorPIDConstants(deviceserial_t serial, int32_t kP, int32_t kI, int32_t kD);

/**
 * @brief Set the mode of a motor board.
 *
 * @param serial The CAN serial number of the motor board.
 * @param mode The mode to set.
 */
void setMotorMode(deviceserial_t serial, motormode_t mode);

/**
 * @brief Set the power output of a motor board.
 *
 * @param serial The CAN serial number of the motor board.
 * @param power Percent power, in the range [-1,1].
 */
void setMotorPower(deviceserial_t serial, double power);

/**
 * @brief Set the power output of a motor board.
 *
 * The motor mode should have been set to motormode_t::pwm.
 *
 * @param serial The CAN serial number of the motor board.
 * @param power The power to set. Any signed 16-bit integer is valid.
 */
void setMotorPower(deviceserial_t serial, int16_t power);

/**
 * @brief Set the position PID target of a motor board.
 *
 * The motor mode should have been set to motormode_t::pid.
 * Additionally, both the sensor and the PID coefficients must have been initialized.
 *
 * @param serial The CAN serial number of the motor board.
 * @param target The position in millidegrees to track with the PID controller.
 */
void setMotorPIDTarget(deviceserial_t serial, int32_t target);

/**
 * @brief Set the maximum power allowed for a motor when running PID.
 *
 * @param serial The CAN serial number of the motor board.
 * @param maxPower The maximum power allowed for the motor.
 */
void setMotorPIDMaxPower(deviceserial_t serial, uint16_t maxPower);

/**
 * @brief Set the angle of the PCA servo
 *
 * @param serial The CAN serial number of the motor board.
 * @param servoNum the servo number.
 * @param angle the angle of the servo in millidegrees.
 */
void setServoPos(deviceserial_t serial, uint8_t servoNum, int32_t angle);

/**
 * @brief Get the last reported position of a motor.
 *
 * This only reports the cached position, it does not poll the motor board for new data.
 *
 * @param serial The serial number of the motor board.
 * @return robot::types::DataPoint<int32_t> The position data of the given motor, in
 * millidegrees. If no position data has been received, returns an empty data point.
 */
robot::types::DataPoint<int32_t> getMotorPosition(deviceserial_t serial);

/**
 * @brief Poll the position data from a motor board.
 *
 * This may not be supported by every motor board implementation.
 *
 * @param serial The CAN serial number of the motor board.
 */
void pullMotorPosition(deviceserial_t serial);

/**
 * @brief Add a callback that is invoked when the limit switch is triggered for a motor board.
 *
 * The event is only triggered when the limit switch is clicked, not released.
 *
 * @param serial The CAN serial number of the motor board.
 * @param callback The callback to invoke when the limit switch is triggered.
 * @return callbackid_t An ID that refers to this callback.
 * This can be passed to removeLimitSwitchCallback() to remove this callback.
 */
callbackid_t addLimitSwitchCallback(
	deviceserial_t serial,
	const std::function<void(
		deviceserial_t serial,
		robot::types::DataPoint<robot::types::LimitSwitchData> limitSwitchData)>& callback);

/**
 * @brief Remove a previously registered limit switch callback.
 *
 * @param id The callback ID that was returned when the callback was registered with
 * addLimitSwitchCallback().
 */
void removeLimitSwitchCallback(callbackid_t id);
} // namespace can::motor
