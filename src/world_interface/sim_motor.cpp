#include "sim_motor.h"

#include "../Globals.h"
#include "world_interface.h"

namespace robot {
class sim_motor : public base_motor {
public:
	sim_motor(robot::types::motorid_t motor, bool hasPosSensor, const std::string& name,
			  const std::string& path) : base_motor(motor, hasPosSensor), motor_name(name), protocol_path(path) {}

	void setMotorPower(double power) {
		// unschedule velocity event if exists
		resetEventID();

		json msg = {{"type", "simMotorPowerRequest"}, {"motor", motor_name}, {"power", power}};
		sendJSON(msg);
	}

	void setMotorPos(int32_t targetPos) {
		// unschedule velocity event if exists
		resetEventID();

		json msg = {{"type", "simMotorPositionRequest"},
					{"motor", motor_name},
					{"position", targetPos}};
		sendJSON(msg);
	}

	types::DataPoint<int32_t> getMotorPos() const {
		robot::getMotorPos(motor_id);
	}

private:
	std::string motor_name;
	std::string protocol_path;

	void sendJSON(const json& obj) {
		Globals::websocketServer.sendJSON(protocol_path, obj);
	}
}; // class sim_motor
} // namespace robot