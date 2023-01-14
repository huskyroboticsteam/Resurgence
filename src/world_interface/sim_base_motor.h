#pragma once

#include "../world_interface/base_motor.h"
#include "../network/websocket/WebSocketProtocol.h"
#include <nlohmann/json.hpp>

using nlohmann::json;

/**
 * @namespace robot
 * @brief Collection of functions for manipulating a motor.
 */
namespace robot {
class sim_motor: public base_motor {
    public: 
        /**
         * @brief Constructor for can motor.
         *
         * @param motor The motor to manipulate.
         */
		sim_motor(robot::types::motorid_t motor, std::string name, std::string protocol_path) {}

		void setPositionSensor(bool hasPosSensor) {}

		bool hasPositionSensor() {}

		void setMotorPower(robot::types::motorid_t motor, double power) {}

		void setMotorPos(robot::types::motorid_t motor, int32_t targetPos) {}

        types::DataPoint<int32_t> getMotorPos(robot::types::motorid_t motor) {}

		void setMotorVel(robot::types::motorid_t motor, int32_t targetVel) {}

    private:
        std::string motor_name;
        std::string protocol_path;
        void sendJSON(const json& obj) {}
}; // class sim_motor
} // namespace robot