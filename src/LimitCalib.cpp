#include "utils/threading.h"
#include "world_interface/real_world_constants.h"
#include "world_interface/world_interface.h"

using namespace robot;
using robot::types::DataPoint;
using robot::types::LimitSwitchData;

namespace {

util::latch latch(encMotors.size());

void callback(motorid_t motor, const types::DataPoint<LimitSwitchData>& data) {
	setMotorPower(motor, 0);
	// call countdown.
	latch.count_down();
};

} // namespace

// Runs limit switch calibration.
// Registers limit switch callbacks for the relevant motors.
// Then begins to run the motors.
int main() {
	bool runCalibration = true;
	// init motors.
	robot::world_interface_init(runCalibration);

	// limit switch callbacks.
	for (const auto& entry : encMotors) {
		robot::addLimitSwitchCallback(entry.first, callback);
	}

	// run motors to get to limit switch.
	for (const auto& runningMotor : encMotors) {
		setMotorPower(runningMotor.first, -0.2);
	}

	// wait for latch to unlatch.
	latch.wait();
}
