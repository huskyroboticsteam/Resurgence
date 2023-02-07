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
         * @param motor The motor id to manipulate.
         * @param hasPosSensor Boolean to indicate if the motor has a position sensor.
         */
		can_motor(robot::types::motorid_t motor, bool hasPosSensor);

		bool hasPositionSensor() const;

		void setMotorPower(double power);

		void setMotorPos(int32_t targetPos);

        types::DataPoint<int32_t> getMotorPos() const;

    private:
        void constructVelController();
}; // class can
} // namespace robot