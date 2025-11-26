#pragma once

#include "../../network/websocket/WebSocketProtocol.h"
#include "base_motor.h"

#include <nlohmann/json.hpp>

/**
 * @namespace robot
 * @brief Collection of functions for manipulating a motor.
 */
namespace robot {
class sim_motor : public base_motor {
public:
	/**
	 * @brief Constructor for can motor.
	 *
	 * @param motor The motor id to manipulate.
	 * @param hasPosSensor Boolean to indicate if the motor has a position sensor.
	 * @param name The name of the motor.
	 * @param protocol_path Websocket protocol path.
	 */
	sim_motor(robot::types::boardid_t motor, bool hasPosSensor, const std::string& name,
			  const std::string& path);

	void setMotorPower(double power) override;

	void setMotorPos(int32_t targetPos) override;

	types::DataPoint<int32_t> getMotorPos() const override;

private:
	std::string motor_name;
	std::string protocol_path;
	void sendJSON(const nlohmann::json& obj);
}; // class sim_motor
} // namespace robot
