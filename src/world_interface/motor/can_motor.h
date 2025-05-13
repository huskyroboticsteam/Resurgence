#pragma once

#include "../../CAN/CAN.h"
#include "../../CAN/CANMotor.h"
#include "../../CAN/CANUtils.h"
#include "base_motor.h"

/**
 * @namespace robot
 * @brief Collection of functions for manipulating a motor.
 */
namespace robot {
class can_motor : public base_motor {
public:
	/**
	 * @brief Constructor for can motor.
	 *
	 * @param motor The motor id to manipulate.
	 * @param hasPosSensor Boolean to indicate if the motor has a position sensor.
	 * @param serial The serial number of the motor board.
	 * @param pos_pwm_scale The positive pwm scale for the motor power.
	 * @param neg_pwm_scale The positive pwm scale for the motor power.
	 */
	can_motor(robot::types::motorid_t motor, bool hasPosSensor, can::deviceserial_t serial,
			  can::devicegroup_t group, double pos_pwm_scale, double neg_pwm_scale);

	void setMotorPower(double power) override;

	void setMotorPos(int32_t targetPos) override;

	types::DataPoint<int32_t> getMotorPos() const override;

	void setMotorVel(int32_t targetVel) override;

  void setServoPos(robot::types::servoid_t servo, int32_t position);

	/**
	 * @brief Returns the motor's serial id.
	 */
	can::deviceserial_t getMotorSerial();

private:
	can::deviceserial_t serial_id;
	can::devicegroup_t device_group;
	std::optional<can::motor::motormode_t> motor_mode;
	double positive_scale;
	double negative_scale;
	void ensureMotorMode(robot::types::motorid_t motor, can::motor::motormode_t mode);
}; // class can_motor
} // namespace robot