#include "can_motor.h"

#include "../../CAN/CAN.h"
#include "../../CAN/CANMotor.h"
#include "../../CAN/CANUtils.h"

namespace robot {
can_motor::can_motor(robot::types::motorid_t motor, bool hasPosSensor, can::deviceserial_t serial, 
					frozen::unordered_map<robot::types::motorid_t, double, 10UL> pos_pwm_scales, 
					frozen::unordered_map<robot::types::motorid_t, double, 10UL> neg_pwm_scales)
	: base_motor(motor, hasPosSensor), serial_id(serial), positive_scales(pos_pwm_scales), negative_scales(neg_pwm_scales) {}

void can_motor::setMotorPower(double power) {
	ensureMotorMode(motor_id, can::motor::motormode_t::pwm);

	// unschedule velocity event if exists
	unscheduleVelocityEvent();

	auto& scaleMap = power < 0 ? negative_scales : positive_scales;
	auto entry = scaleMap.find(motor_id);
	if (entry != scaleMap.end()) {
		power *= entry->second;
	}
	can::motor::setMotorPower(serial_id, power);
}

void can_motor::setMotorPos(int32_t targetPos) {
	ensureMotorMode(motor_id, can::motor::motormode_t::pid);

	// unschedule velocity event if exists
	unscheduleVelocityEvent();

	can::motor::setMotorPIDTarget(serial_id, targetPos);
}

types::DataPoint<int32_t> can_motor::getMotorPos() const {
	return can::motor::getMotorPosition(serial_id);
}

void can_motor::setMotorVel(int32_t targetVel) {
	ensureMotorMode(motor_id, can::motor::motormode_t::pid);
	base_motor::setMotorVel(targetVel);
}

can::deviceserial_t can_motor::getMotorSerial() {
	return serial_id;
}

void can_motor::ensureMotorMode(robot::types::motorid_t motor, can::motor::motormode_t mode) {
	if (!motor_mode) {
		motor_mode.emplace(mode);
	} else if (motor_mode.value() != mode) {
		motor_mode.emplace(mode);
		can::motor::setMotorMode(serial_id, mode);
	}
}
} // namespace robot
