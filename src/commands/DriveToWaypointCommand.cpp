#include "DriveToWaypointCommand.h"

#include <loguru.hpp>

using namespace navtypes;
using namespace robot::types;
using namespace std::chrono_literals;

namespace commands {

DriveToWaypointCommand::DriveToWaypointCommand(const point_t& target, double thetaKP,
											   double driveVel, double slowDriveThresh,
											   double doneThresh,
											   util::dseconds closeToTargetDur)
	: target(target), pose(pose_t::Zero()), thetaKP(thetaKP), driveVel(driveVel),
	  slowDriveThresh(slowDriveThresh), doneThresh(doneThresh),
	  setStateCalledBeforeOutput(false), closeToTargetDur(closeToTargetDur) {}

void DriveToWaypointCommand::setState(const navtypes::pose_t& pose,
									  robot::types::datatime_t time) {
	this->pose = pose;
	this->setStateCalledBeforeOutput = true;
	this->lastRecordedTime = time;
}

command_t DriveToWaypointCommand::getOutput() {
	if (!this->setStateCalledBeforeOutput) {
		LOG_F(WARNING, "DriveToWaypointCommand: getOutput() called before getState() call!");
	}

	this->setStateCalledBeforeOutput = false;

	double thetaVel = 0;
	double dist = (pose.topRows<2>() - target.topRows<2>()).norm();
	if (dist > slowDriveThresh) {
		Eigen::Vector2d toTarget = target.topRows<2>() - pose.topRows<2>();
		double targetAngle = std::atan2(toTarget(1), toTarget(0));
		double angleDelta = targetAngle - pose(2);
		double thetaErr = std::atan2(std::sin(angleDelta), std::cos(angleDelta));
		double thetaVel = thetaKP * thetaErr;
	}
	
	return {.thetaVel = thetaVel, .xVel = driveVel};
}

bool DriveToWaypointCommand::isDone() {
	if (!lastRecordedTime.has_value()) {
		return false;
	}

	double distance = (pose.topRows<2>() - target.topRows<2>()).norm();
	if (distance <= doneThresh) {
		if (!closeToTargetStartTime.has_value()) {
			closeToTargetStartTime = lastRecordedTime;
		}

		return lastRecordedTime.value() - closeToTargetStartTime.value() >= closeToTargetDur;
	} else {
		closeToTargetStartTime.reset();
	}
	return false;
}

} // namespace commands
