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
         * @param motor The motor id to manipulate.
         * @param hasPosSensor Boolean to indicate if the motor has a position sensor.
         * @param name The name of the motor.
         * @param protocol_path Websocket protocol path.
         */
		sim_motor(robot::types::motorid_t motor, bool hasPosSensor, std::string name, std::string protocol_path) {}

		bool hasPositionSensor() const {}

		void setMotorPower(double power) {}

		void setMotorPos(int32_t targetPos) {}

        types::DataPoint<int32_t> getMotorPos() const {}

		void setMotorVel(int32_t targetVel) {}

    private:
        std::string motor_name;
        std::string protocol_path;
        void sendJSON(const json& obj) {}
}; // class sim_motor
} // namespace robot