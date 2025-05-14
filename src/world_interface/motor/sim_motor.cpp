#include "sim_motor.h"

#include "../../Globals.h"
#include "../world_interface.h"

using nlohmann::json;

namespace robot {
sim_motor::sim_motor(robot::types::motorid_t motor, bool hasPosSensor, const std::string& name,
					 const std::string& path)
	: base_motor(motor, hasPosSensor), motor_name(name), protocol_path(path) {}

void sim_motor::setMotorPower(double power) {
	// unschedule velocity event if exists
	unscheduleVelocityEvent();

	json msg = {{"type", "simMotorPowerRequest"}, {"motor", motor_name}, {"power", power}};
	sendJSON(msg);
}

void sim_motor::setMotorPos(int32_t targetPos) {
	// unschedule velocity event if exists
	unscheduleVelocityEvent();

	json msg = {
		{"type", "simMotorPositionRequest"}, {"motor", motor_name}, {"position", targetPos}};
	sendJSON(msg);
}

types::DataPoint<int32_t> sim_motor::getMotorPos() const {
	// this calles the method implementation in world interface
	return robot::getMotorPos(motor_id);
}

// Do we want to leave this as servo number?
void sim_motor::setServoPos(uint8_t servo, int32_t position) {
	json msg = {
		{"type", "simServoPositionRequest", {"servo", servo}, {"position", position}}};
	sendJSON(msg);
}

void sim_motor::sendJSON(const json& obj) {
	Globals::websocketServer.sendJSON(protocol_path, obj);
}
} // namespace robot
