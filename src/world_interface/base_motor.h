#pragma once

#include "data.h"
#include "../control/JacobianVelController.h"

/**
 * @namespace robot
 * @brief Collection of functions for manipulating a motor.
 */
namespace robot {
/**
 * An abstract motor class
 * 
 */
class base_motor {
public:
    /**
     * @brief Default constructor for base motor.
     *
     */
    base_motor();

    /**
     * @brief Returns the status of the motor position sensor.
     *
     * @return True if the motor has a position sensor and false if not
     */
    virtual bool hasPositionSensor() const = 0;

    /**
     * @brief Set the PWM command of the motor.
     *
     * @param power The power command, in the range [-1, 1]
     */
    virtual void setMotorPower(double power) = 0;

    /**
     * @brief Set the target position of the motor. This will have no effect if the motor
     * does not support PID.
     *
     * @param targetPos The target position, in millidegrees. Refer to the specific motor for more
     * information.
     */
    virtual void setMotorPos(int32_t targetPos) = 0;

    /**
     * @brief Get the last reported position of the specified motor.
     *
     * @return types::DataPoint<int32_t> The last reported position of the motor, if it exists.
     * If the motor has not reported a position (because it hasn't been received yet or if it
     * doesn't have an encoder) then an empty data point is returned.
     */
    virtual types::DataPoint<int32_t> getMotorPos() const = 0;

    /**
     * @brief Sets the velocity of the motor.
     *
     * @param targetVel The target velocity, in millidegrees per second.
     */
    virtual void setMotorVel(int32_t targetVel) = 0;

protected:
    robot::types::motorid_t motor_id;
    bool posSensor;
}; // abstract class base_motor
} // namespace robot