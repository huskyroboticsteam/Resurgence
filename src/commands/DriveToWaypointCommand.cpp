#include "DriveToWaypointCommand.h"

#include <loguru.hpp>

using namespace navtypes;
using namespace robot::types;
using namespace std::chrono_literals;

namespace commands {

DriveToWaypointCommand::DriveToWaypointCommand(const point_t& target, double thetaKP,
											   double driveVel, double doneThresh)
					: target(target), pose(pose_t::Zero()), thetaKP(thetaKP), driveVel(driveVel),
					  doneThresh(doneThresh), setStateCalledBeforeOutput(false) {}

void DriveToWaypointCommand::setState(const navtypes::pose_t& pose) {
	this->pose = pose;
	this->setStateCalledBeforeOutput = true;
}

command_t DriveToWaypointCommand::getOutput() {
	if (!this->setStateCalledBeforeOutput) {
		LOG_F(WARNING, "DriveToWaypointCommand: getOutput() called before getState() call!");
	}

	this->setStateCalledBeforeOutput = false;
	Eigen::Vector2d toTarget = target.topRows<2>() - pose.topRows<2>();
	double targetAngle = std::atan2(toTarget(1), toTarget(0));
	double angleDelta = targetAngle - pose(2);

	// p-controller (https://x-engineer.org/proportional-controller/)
	// wrap angleDelta to [-pi, pi] to prevent 90+ degree turns
	angleDelta = std::atan2(std::sin(angleDelta), std::cos(angleDelta));

	double thetaVel = thetaKP * angleDelta;

	double dist = (pose.topRows<2>() - target.topRows<2>()).norm();
	double xVel = driveVel;
	return {.thetaVel = thetaVel, .xVel = xVel};
}

bool DriveToWaypointCommand::isDone() {
	double distance = (pose.topRows<2>() - target.topRows<2>()).norm();
	return distance <= doneThresh;
}

} // namespace commands
