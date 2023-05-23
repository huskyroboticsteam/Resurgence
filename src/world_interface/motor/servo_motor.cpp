#include "servo_motor.h"

namespace robot {
servo_motor::servo_motor(robot::types::motorid_t motor, bool hasPosSensor,
						 can::deviceserial_t serial, std::vector<uint8_t> indices)
	: base_motor(motor, hasPosSensor), serial_id(serial), servo_indices(indices) {}

void servo_motor::setMotorPower(double power, int32_t maxVelocity) {
	setMotorVel(power * maxVelocity);
}

void servo_motor::setMotorPos(int32_t targetPosition) {
	for (uint8_t servoNum : servo_indices) {
		can::motor::setServoPos(serial_id, servoNum, targetPosition);
	}
}
} // namespace robot
