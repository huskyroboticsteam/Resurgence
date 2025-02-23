#include "can_motor.h"

namespace robot {
can_motor::can_motor(robot::types::motorid_t motor, bool hasPosSensor,
					 can::deviceserial_t serial, can::devicegroup_t group,
					 double pos_pwm_scale, double neg_pwm_scale)
	: base_motor(motor, hasPosSensor), serial_id(serial), device_group(group),
	  positive_scale(pos_pwm_scale), negative_scale(neg_pwm_scale) {}

void can_motor::setMotorPower(double power) {
	ensureMotorMode(motor_id, can::motor::motormode_t::pwm);

	// unschedule velocity event if exists
	unscheduleVelocityEvent();

	// scale the power
	double scale = power < 0 ? negative_scale : positive_scale;
	power *= scale;
	can::motor::setMotorPower(device_group, serial_id, power);
}

void can_motor::setMotorPos(int32_t targetPos) {
	ensureMotorMode(motor_id, can::motor::motormode_t::pid);

	// unschedule velocity event if exists
	unscheduleVelocityEvent();

	can::motor::setMotorPIDTarget(device_group, serial_id, targetPos);
}

types::DataPoint<int32_t> can_motor::getMotorPos() const {
	return can::motor::getMotorPosition(device_group, serial_id);
}

void can_motor::setMotorVel(int32_t targetVel) {
	ensureMotorMode(motor_id, can::motor::motormode_t::pid);
	base_motor::setMotorVel(targetVel);
}

can::deviceserial_t can_motor::getMotorSerial() {
	return device_id.second;
}

void can_motor::ensureMotorMode(robot::types::motorid_t motor, can::motor::motormode_t mode) {
	if (!motor_mode || motor_mode.value() != mode) {
		// update the motor mode
		motor_mode.emplace(mode);
		can::motor::setMotorMode(device_group, serial_id, mode);
	}
}
} // namespace robot
