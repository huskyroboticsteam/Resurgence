#pragma once

#include "../world_interface/data.h"
#include "CAN.h"
#include "CANUtils.h"

#include <chrono>
#include <optional>

extern "C" {
#include <HindsightCAN/CANCommon.h> // Include for ODrive CANSimple functions
}

/**
 * @namespace can::motor
 * @brief Utilities for interacting with motor boards over a CAN interface.
 *
 * These motor boards can either be AVR, PSoC, or ODrive boards.
 */
namespace can::motor {

/** @brief The possible motor modes for AVR/PSoC boards. */
enum class motormode_t { pwm = MOTOR_UNIT_MODE_PWM, pid = MOTOR_UNIT_MODE_PID };

/** @brief The possible motor modes for ODrive boards. */
enum class odrive_motormode_t {
    position = 0,  // Position control mode
    velocity = 1,  // Velocity control mode
    torque = 2     // Torque control mode
};

/** @brief The supported motor position sensors for AVR/PSoC boards. */
struct sensor_t {
    enum { encoder = 0, potentiometer = 1 };
};

/**
 * @brief Emergency stop all motors on the CAN bus.
 *
 * This cuts power to all motors and resets them, including ODrive motors.
 */
void emergencyStopMotors();

/**
 * @brief Initialize an AVR/PSoC motor.
 *
 * This does not initialize any sensor or controller.
 *
 * @param serial The CAN serial number of the motor to initialize.
 */
void initMotor(deviceserial_t serial);

/**
 * @brief Initialize an ODrive motor.
 *
 * This sets up the ODrive motor with initial configurations (e.g., axis state, controller mode).
 *
 * @param node_id The ODrive node ID (e.g., NODE_ID_1 or NODE_ID_2).
 */
void initODriveMotor(uint8_t node_id);

/**
 * @brief Initialize an encoder attached to an AVR/PSoC motor.
 *
 * For potentiometers, use initPotentiometer().
 *
 * @param serial The CAN serial number of the motor board.
 * @param invertEncoder If true, invert the encoder direction.
 * @param zeroEncoder If true, reset the encoder position to zero.
 * @param pulsesPerJointRev The number of encoder pulses per revolution of the joint.
 * @param telemetryPeriod An optional parameter specifying the telemetry period.
 */
void initEncoder(deviceserial_t serial, bool invertEncoder, bool zeroEncoder,
                 int32_t pulsesPerJointRev,
                 std::optional<std::chrono::milliseconds> telemetryPeriod);

/**
 * @brief Set the limits of the limit switch on an AVR/PSoC motor board.
 *
 * @param serial The CAN serial number of the motor board.
 * @param lo The joint position in millidegrees of the low limit switch.
 * @param hi The joint position in millidegrees of the high limit switch.
 */
void setLimitSwitchLimits(deviceserial_t serial, int32_t lo, int32_t hi);

/**
 * @brief Initialize a potentiometer attached to an AVR/PSoC motor.
 *
 * @param serial The CAN serial number of the motor board.
 * @param posLo The joint position that corresponds to adcLo.
 * @param posHi The joint position that corresponds to adcHi.
 * @param adcLo The ADC value when the joint is at posLo.
 * @param adcHi The ADC value when the joint is at posHi.
 * @param telemetryPeriod An optional parameter specifying the telemetry period.
 */
void initPotentiometer(deviceserial_t serial, int32_t posLo, int32_t posHi, uint16_t adcLo,
                       uint16_t adcHi,
                       std::optional<std::chrono::milliseconds> telemetryPeriod);

/**
 * @brief Set the PID constants for an AVR/PSoC motor board.
 *
 * @param serial The CAN serial number of the motor board.
 * @param kP The P coefficient (in units of 10000).
 * @param kI The I coefficient (in units of 10000).
 * @param kD The D coefficient (in units of 10000).
 */
void setMotorPIDConstants(deviceserial_t serial, int32_t kP, int32_t kI, int32_t kD);

/**
 * @brief Set the PID gains for an ODrive motor.
 *
 * @param node_id The ODrive node ID.
 * @param pos_gain Position gain for position control mode.
 * @param vel_gain Velocity gain for velocity control mode.
 * @param vel_integrator_gain Velocity integrator gain for velocity control mode.
 */
void setODrivePIDGains(uint8_t node_id, float pos_gain, float vel_gain, float vel_integrator_gain);

/**
 * @brief Set the mode of an AVR/PSoC motor board.
 *
 * @param serial The CAN serial number of the motor board.
 * @param mode The mode to set.
 */
void setMotorMode(deviceserial_t serial, motormode_t mode);

/**
 * @brief Set the mode of an ODrive motor.
 *
 * @param node_id The ODrive node ID.
 * @param mode The mode to set (position, velocity, or torque).
 */
void setODriveMode(uint8_t node_id, odrive_motormode_t mode);

/**
 * @brief Set the power output of an AVR/PSoC motor board (percent power).
 *
 * @param serial The CAN serial number of the motor board.
 * @param power Percent power, in the range [-1,1].
 */
void setMotorPower(deviceserial_t serial, double power);

/**
 * @brief Set the power output of an AVR/PSoC motor board (raw PWM).
 *
 * @param serial The CAN serial number of the motor board.
 * @param power The power to set (signed 16-bit integer).
 */
void setMotorPower(deviceserial_t serial, int16_t power);

/**
 * @brief Set the position PID target of an AVR/PSoC motor board.
 *
 * @param serial The CAN serial number of the motor board.
 * @param target The position in millidegrees to track.
 */
void setMotorPIDTarget(deviceserial_t serial, int32_t target);

/**
 * @brief Set the position target for an ODrive motor in position control mode.
 *
 * @param node_id The ODrive node ID.
 * @param position The target position (in turns, as per ODrive units).
 * @param vel_ff Velocity feedforward (in turns/s).
 * @param torque_ff Torque feedforward (in Nm).
 */
void setODrivePosition(uint8_t node_id, float position, float vel_ff = 0.0f, float torque_ff = 0.0f);

/**
 * @brief Set the velocity target for an ODrive motor in velocity control mode.
 *
 * @param node_id The ODrive node ID.
 * @param velocity The target velocity (in turns/s).
 * @param torque_ff Torque feedforward (in Nm).
 */
void setODriveVelocity(uint8_t node_id, float velocity, float torque_ff = 0.0f);

/**
 * @brief Set the torque target for an ODrive motor in torque control mode.
 *
 * @param node_id The ODrive node ID.
 * @param torque The target torque (in Nm).
 */
void setODriveTorque(uint8_t node_id, float torque);

/**
 * @brief Set the angle of the PCA servo on an AVR/PSoC motor board.
 *
 * @param serial The CAN serial number of the motor board.
 * @param servoNum The servo number.
 * @param angle The angle of the servo in millidegrees.
 */
void setServoPos(deviceserial_t serial, uint8_t servoNum, int32_t angle);

/**
 * @brief Get the last reported position of an AVR/PSoC motor.
 *
 * @param serial The CAN serial number of the motor board.
 * @return DataPoint<int32_t> The position data in millidegrees.
 */
DataPoint<int32_t> getMotorPosition(deviceserial_t serial);

/**
 * @brief Get the last reported position of an ODrive motor.
 *
 * @param node_id The ODrive node ID.
 * @return DataPoint<float> The position data in turns.
 */
DataPoint<float> getODrivePosition(uint8_t node_id);

/**
 * @brief Get the last reported velocity of an ODrive motor.
 *
 * @param node_id The ODrive node ID.
 * @return DataPoint<float> The velocity data in turns/s.
 */
DataPoint<float> getODriveVelocity(uint8_t node_id);

/**
 * @brief Get the last reported temperature of an ODrive motor.
 *
 * @param node_id The ODrive node ID.
 * @return DataPoint<std::pair<float, float>> The FET and motor temperatures in Celsius.
 */
DataPoint<std::pair<float, float>> getODriveTemperature(uint8_t node_id);

/**
 * @brief Poll the position data from an AVR/PSoC motor board.
 *
 * @param serial The CAN serial number of the motor board.
 */
void pullMotorPosition(deviceserial_t serial);

/**
 * @brief Poll the encoder estimates (position and velocity) from an ODrive motor.
 *
 * @param node_id The ODrive node ID.
 */
void pullODriveEncoderEstimates(uint8_t node_id);

/**
 * @brief Poll the temperature data from an ODrive motor.
 *
 * @param node_id The ODrive node ID.
 */
void pullODriveTemperature(uint8_t node_id);

/**
 * @brief Add a callback for limit switch events on an AVR/PSoC motor board.
 *
 * @param serial The CAN serial number of the motor board.
 * @param callback The callback to invoke when the limit switch is triggered.
 * @return callbackid_t An ID for the callback.
 */
callbackid_t addLimitSwitchCallback(
    deviceserial_t serial,
    const std::function<void(deviceserial_t serial,
                             DataPoint<LimitSwitchData> limitSwitchData)>& callback);

/**
 * @brief Remove a previously registered limit switch callback.
 *
 * @param id The callback ID.
 */
void removeLimitSwitchCallback(callbackid_t id);
} // namespace can::motor
