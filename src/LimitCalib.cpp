#include "control/TrapezoidalVelocityProfile.h"
#include "log.h"
#include "utils/threading.h"
#include "world_interface/real_world_constants.h"
#include "world_interface/world_interface.h"

#include <chrono>
#include <csignal>
#include <iostream>

using namespace Eigen;
using namespace robot;
using namespace std::chrono;
using robot::types::DataPoint;
using robot::types::LimitSwitchData;

namespace {

util::latch latch(encMotors.size());

void callback(motorid_t motor, const types::DataPoint<LimitSwitchData>& data) {
	robot::setMotorPower(motor, 0);
	// call countdown.
	latch.count_down();
};

void cleanup(int signum) {
	log(LOG_ERROR, "Interrupted!\n");
	robot::emergencyStop();
	exit(0);
}

} // namespace

// Runs limit switch calibration.
// Registers limit switch callbacks for the relevant motors.
// Then begins to run the motors.
int main() {
	// init motors.
	robot::world_interface_init(true);

	signal(SIGINT, cleanup);

	// limit switch callbacks.
	for (const auto& entry : encMotors) {
		robot::addLimitSwitchCallback(entry.first, callback);
	}

	log(LOG_INFO, "Zero calibrating...\n");

	// run motors until latch unlatches
	do {
		for (const auto& runningMotor : encMotors) {
			robot::setMotorPower(runningMotor.first, runningMotor.second.zeroCalibrationPower);
		}
	} while (!latch.wait_for(500ms));

	log(LOG_INFO, "Done\n");

	constexpr std::array<robot::types::motorid_t, 2> sortedKeys = {
		{robot::types::motorid_t::shoulder, robot::types::motorid_t::elbow}};

	std::sort(sortedKeys.begin(), sortedKeys.end());

	control::TrapezoidalVelocityProfile<encMotors.size()> profile(0.4, 0.6);
	// std::vector<double, 2> targetPosVec;
	navtypes::Vectord<2> targetPosVec;
	std::array<robot::types::motorid_t, 2> motorNames;

	int i = 0;
	for (const auto& runningMotor : encMotors) {
		motorNames[i] = runningMotor.first;
		targetPosVec[i] = runningMotor.second.stdPos;
		i++;
	}

	// std::sort(targetPosVec.begin(), targetPosVec.end(), customVecComparator);

	robot::types::datatime_t currTime(0s);
	profile.setTarget(currTime, getMotorPositionsRad(motorNames), targetPosVec);

	JacobianVelController<2, 2> controller([](Eigen::VectorXd x) { return x; }, {});

	double d;
	navtypes::Vectord<2> targetVel = profile.getCommand(currTime);
	navtypes::Vectord<2> targetPos = controller.getCommand(currTime, targetVel, d);
	i = 0;
	for (const auto& element : sortedKeys) {
		robot::setMotorPos(element, targetPos[i]);
		i++;
	}
}
