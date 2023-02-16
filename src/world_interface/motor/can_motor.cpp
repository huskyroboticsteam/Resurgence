#include "can_motor.h"

#include "../../CAN/CAN.h"
#include "../../CAN/CANMotor.h"
#include "../../CAN/CANUtils.h"
#include "../real_world_constants.h"

namespace robot {
class can_motor : public base_motor {
public:
	can_motor(robot::types::motorid_t motor, bool hasPosSensor)
		: base_motor(motor, hasPosSensor) {}

	void setMotorPower(double power) {
		// unschedule velocity event if exists
		unscheduleVelocityEvent();

		can::deviceserial_t serial = motorSerialIDMap.at(motor_id);
		auto& scaleMap = power < 0 ? negative_pwm_scales : positive_pwm_scales;
		auto entry = scaleMap.find(motor_id);
		if (entry != scaleMap.end()) {
			power *= entry->second;
		}
		can::motor::setMotorPower(serial, power);
	}

	void setMotorPos(int32_t targetPos) {
		// unschedule velocity event if exists
		unscheduleVelocityEvent();

		can::deviceserial_t serial = motorSerialIDMap.at(motor_id);
		can::motor::setMotorPIDTarget(serial, targetPos);
	}

	types::DataPoint<int32_t> getMotorPos() const {
		return can::motor::getMotorPosition(motorSerialIDMap.at(motor_id));
	}
}; // class can_motor
} // namespace robot
