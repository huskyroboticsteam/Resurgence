#pragma once

#include "../world_interface/base_motor.h"

/**
 * @namespace robot
 * @brief Collection of functions for manipulating a motor.
 */
namespace robot {
class can_motor: public base_motor {
    public: 
        /**
         * @brief Constructor for can motor.
         *
         * @param motor The motor to manipulate.
         */
		can_motor(robot::types::motorid_t motor) {}

		void setPositionSensor(bool hasPosSensor) {}

		bool hasPositionSensor() {}

		void setMotorPower(robot::types::motorid_t motor, double power) {}

		void setMotorPos(robot::types::motorid_t motor, int32_t targetPos) {}

        types::DataPoint<int32_t> getMotorPos(robot::types::motorid_t motor) {}

		void setMotorVel(robot::types::motorid_t motor, int32_t targetVel) {}

}; // class can_motor
} // namespace robot