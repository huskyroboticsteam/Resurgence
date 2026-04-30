#pragma once

#include "../world_interface/data.h"
#include "../control/JacobianVelController.h"
#include "CAN.h"
#include "CANUtils.h"

#include <chrono>
#include <mutex>
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
	vel = BLDC_VELOCITY_CONTROL,
	pos = BLDC_POSITION_CONTROL
};
/*
enum class motormode_t {
	pwm = MOTOR_UNIT_MODE_PWM,
	pid = MOTOR_UNIT_MODE_PID
};
*/

enum class motorstate_t {
    idle = BLDC_AXIS_IDLE,
    control = BLDC_AXIS_CLOSED_LOOP_CONTROL,
};

class CANBoard {
public:
    CANBoard(robot::types::boardid_t motor, bool hasPosSensor, CANDevice_t device,
             double pos_pwm_scale, double neg_pwm_scale);

    void setMotorPower(double power);
    void setMotorPos(int32_t targetPos);
    robot::types::DataPoint<int32_t> getMotorPos() const;
    void setMotorVel(int32_t targetVel);
    void unscheduleVelocityEvent();

    can::uuid_t getMotorUUID() const;
    robot::types::boardid_t getBoardID() const;

private:
    robot::types::boardid_t board_id;
    bool has_pos_sensor;
    CANDevice_t device;
    std::optional<motormode_t> motor_mode;
    std::optional<motorstate_t> motor_state;
    double positive_scale;
    double negative_scale;
    std::optional<util::PeriodicScheduler<std::chrono::steady_clock>::eventid_t> velEventID;
    std::optional<JacobianVelController<1, 1>> velController;

    inline static std::optional<util::PeriodicScheduler<std::chrono::steady_clock>> pSched;
    inline static std::mutex schedulerMutex;

    void ensureMotorMode(motormode_t mode);
    void ensureMotorMode(motormode_t mode, motorstate_t state);
    void constructVelController();
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
 * @brief Remove a previously registered limit switch callback.
 *
 * @param id The callback ID that was returned when the callback was registered with
 * addLimitSwitchCallback().
 */
void removeLimitSwitchCallback(callbackid_t id);

void initEncoder();

/**
 * @brief Initialize a motor using CAN26 protocol.
 * @param device The target CAN device.
 */
void initMotor(CANDevice_t device);

void setMotorState(CANDevice_t device, motorstate_t state);

/**
 * @brief Set the motor mode using CAN26 protocol.
 * @param device The target CAN device.
 * @param mode The motor mode to set.
 */
void setMotorMode(CANDevice_t device, motormode_t mode);

/**
 * @brief Set motor power using CAN26 protocol.
 * @param device The target CAN device.
 * @param power Power level in range [-1.0, 1.0].
 */
void setMotorPower(CANDevice_t device, double power);

/**
 * @brief Set motor power using CAN26 protocol.
 * @param device The target CAN device.
 * @param power Power level as int16_t.
 */
void setMotorPower(CANDevice_t device, int16_t power);

/**
 * @brief Set PID position target using CAN26 protocol.
 * @param device The target CAN device.
 * @param target Target position in millidegrees.
 */
void setMotorPIDTarget(CANDevice_t device, int32_t target);

/**
 * @brief Get the last reported motor position.
 * @param device The target CAN device.
 * @return The cached position data, or empty if not available.
 */
robot::types::DataPoint<int32_t> getMotorPosition(CANDevice_t device);

/**
 * @brief Request encoder position from a motor.
 * @param device The target CAN device.
 */
void pullMotorPosition(CANDevice_t device);

/**
 * @brief Add a callback for limit switch events.
 * @param device The target CAN device.
 * @param callback The callback function.
 * @return Callback ID for removal.
 */
callbackid_t addLimitSwitchCallback(
	CANDevice_t device,
	const std::function<void(
		CANDevice_t device,
		robot::types::DataPoint<robot::types::LimitSwitchData> limitSwitchData)>& callback);
} // namespace can::motor
