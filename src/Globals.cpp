#include "Globals.h"

#include "Constants.h"
#include "CAN/CANUtils.h"
#include "kinematics/FabrikSolver.h"
#include "kinematics/PlanarArmFK.h"
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

kinematics::ArmKinematics<2, Constants::arm::IK_MOTORS.size()> createArmKinematics() {
	auto fk = std::make_shared<kinematics::PlanarArmFK<2>>(getSegLens(), getJointLimits(true),
														   getJointLimits(false));
	auto ik = std::make_shared<kinematics::FabrikSolver2D<2>>(fk, IK_SOLVER_THRESH,
															  IK_SOLVER_MAX_ITER);
	return kinematics::ArmKinematics<2, 2>(fk, ik);
}
} // namespace

namespace Globals {
RoverState curr_state;
net::websocket::SingleClientWSServer websocketServer("DefaultServer",
													 Constants::WS_SERVER_PORT);
std::atomic<bool> AUTONOMOUS = false;
robot::types::mountedperipheral_t mountedPeripheral = robot::types::mountedperipheral_t::none;
can::devicegroup_t deviceGroup = can::devicegroup_t::motor;
const kinematics::DiffWristKinematics wristKinematics;
control::PlanarArmController<2> planarArmController(createArmKinematics(),
													Constants::arm::SAFETY_FACTOR);
std::atomic<bool> armIKEnabled = false;
} // namespace Globals
