#include "Globals.h"

#include "Constants.h"
#include "world_interface/data.h"

#include <atomic>
#include <vector>

using robot::types::motorid_t;
using namespace Constants::arm;

namespace {

navtypes::Vectord<IK_MOTORS.size()> getSegLens() {
	navtypes::Vectord<IK_MOTORS.size()> ret;
	for (std::size_t i = 0; i < IK_MOTORS.size(); i++) {
		ret[i] = SEGMENT_LENGTHS.at(IK_MOTORS[i]);
	}
	return ret;
}

navtypes::Vectord<IK_MOTORS.size()> getJointLimits(bool getLow) {
	navtypes::Vectord<IK_MOTORS.size()> ret;
	for (std::size_t i = 0; i < IK_MOTORS.size(); i++) {
		const auto& limits = JOINT_LIMITS.at(IK_MOTORS[i]);
		ret[i] = getLow ? limits.first : limits.second;
	}
	ret *= M_PI / 180.0 / 1000.0;
	return ret;
}

} // namespace

namespace Globals {
RoverState curr_state;
net::websocket::SingleClientWSServer websocketServer("DefaultServer",
													 Constants::WS_SERVER_PORT);
std::atomic<bool> E_STOP = false;
std::atomic<bool> AUTONOMOUS = false;
robot::types::mountedperipheral_t mountedPeripheral = robot::types::mountedperipheral_t::none;
const kinematics::PlanarArmKinematics<Constants::arm::IK_MOTORS.size()>
	planarArmKinematics(getSegLens(), getJointLimits(true), getJointLimits(false),
						IK_SOLVER_THRESH, IK_SOLVER_MAX_ITER);
control::PlanarArmController<2> planarArmController(planarArmKinematics, Constants::arm::SAFETY_FACTOR);
std::atomic<bool> armIKEnabled = false;
} // namespace Globals
