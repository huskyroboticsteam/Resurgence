#pragma once

#include "../../CAN/CAN.h"
#include "../../CAN/CANMotor.h"
#include "../../CAN/CANUtils.h"
#include "base_motor.h"

#include <vector>

/**
 * @namespace robot
 * @brief Collection of functions for manipulating a motor.
 */
namespace robot {
class servo_motor : public base_motor {
	/**
	 * @brief Constructor for servo motor.
	 *
	 * @param motor The motor id to manipulate.
	 * @param hasPosSensor Boolean to indicate if the motor has a position sensor.
	 * @param serial The serial number of the motor board.
	 * @param indices The indices of the servo motors
	 */
	servo_motor(robot::types::motorid_t motor, bool hasPosSensor, can::deviceserial_t serial,
				std::vector<uint8_t> indices);

	/**
	 * @brief set servo power.
	 */
	void setMotorPower(double power, int32_t maxVelocity);
	/**
	 * @brief set servo position.
	 */
	void setMotorPos(int32_t targetPos) override;

private:
	can::deviceserial_t serial_id;
	std::vector<uint8_t> servo_indices;
}; // class servo_motor
} // namespace robot