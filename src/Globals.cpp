#include "Globals.h"

#include "Constants.h"
#include "kinematics/IKSolver2026.h"
#include "kinematics/ArmFK2026.h"
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

kinematics::ArmKinematics<3, Constants::arm::IK_MOTORS.size()> createArmKinematics() {
	auto fk = std::make_shared<kinematics::ArmFK2026>(getSegLens(), getJointLimits(true),
													  getJointLimits(false));
	auto ik = std::make_shared<kinematics::IKSolver2026>(fk);
	return kinematics::ArmKinematics<3, 3>(fk, ik);
}
} // namespace

namespace Globals {
RoverState curr_state;
net::websocket::SingleClientWSServer websocketServer("DefaultServer",
													 Constants::WS_SERVER_PORT);
std::atomic<bool> AUTONOMOUS = false;
robot::types::mountedperipheral_t mountedPeripheral = robot::types::mountedperipheral_t::none;
const kinematics::DiffWristKinematics wristKinematics;
control::SpatialArmController<3> spatialArmController(createArmKinematics(),
													Constants::arm::SAFETY_FACTOR);
std::atomic<bool> armIKEnabled = false;
} // namespace Globals
