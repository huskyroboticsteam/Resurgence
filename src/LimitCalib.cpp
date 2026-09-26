#include "utils/threading.h"
#include "world_interface/real_world_constants.h"
#include "world_interface/world_interface.h"

#include <chrono>
#include <csignal>
#include <loguru.hpp>

using namespace robot;
using namespace std::chrono;
using robot::types::DataPoint;
using robot::types::LimitSwitchData;

namespace {

util::latch latch(encMotors.size());

void callback(boardid_t motor, const types::DataPoint<LimitSwitchData>& data) {
	robot::setMotorPower(motor, 0);
	// call countdown.
	latch.count_down();
};

void cleanup(int signum) {
	LOG_F(ERROR, "Interrupted!");
	robot::emergencyStop();
	exit(0);
}

} // namespace

// Runs limit switch calibration.
// Registers limit switch callbacks for the relevant motors.
// Then begins to run the motors.
int main() {
	// init motors.
	robot::world_interface_init(std::nullopt, true);

	signal(SIGINT, cleanup);

	// limit switch callbacks.
	for (const auto& entry : encMotors) {
		robot::addLimitSwitchCallback(entry.first, callback);
	}

	LOG_F(INFO, "Zero calibrating...");

	// run motors until latch unlatches
	do {
		for (const auto& runningMotor : encMotors) {
			robot::setMotorPower(runningMotor.first, runningMotor.second.zeroCalibrationPower);
		}
	} while (!latch.wait_for(500ms));

	LOG_F(INFO, "Done");
}
