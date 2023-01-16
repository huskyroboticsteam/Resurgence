#include "DriveToWaypointCommand.h"
#include "../log.h"

using namespace navtypes;
using namespace robot::types;
using namespace std::chrono_literals;

static std::chrono::duration CLOSE_TO_TARGET_DUR = 1s;

namespace commands {

DriveToWaypointCommand::DriveToWaypointCommand(const point_t& target, double thetaKP,
											   double driveVel, double slowDriveVel,
											   double doneThresh)
	: target(target), pose(pose_t::Zero()), thetaKP(thetaKP), driveVel(driveVel),
	  slowDriveVel(slowDriveVel), doneThresh(doneThresh), setStateCalledBeforeOutput(false) {}

void DriveToWaypointCommand::setState(const navtypes::pose_t& pose, 
									  const robot::types::datatime_t time) {
	this->pose = pose;
	this->setStateCalledBeforeOutput = true;
	this->lastRecordedTime = time;
}

command_t DriveToWaypointCommand::getOutput() {
	if (!this->setStateCalledBeforeOutput) {
		log(LOG_WARN, "DriveToWaypointCommand: getOutput() called before getState() call!");
	}

	this->setStateCalledBeforeOutput = false;
	Eigen::Vector2d toTarget = target.topRows<2>() - pose.topRows<2>();
	double targetAngle = std::atan2(toTarget(1), toTarget(0));
	double angleDelta = targetAngle - pose(2);
	double thetaErr =
		std::atan2(std::sin(angleDelta), std::cos(angleDelta));

	double thetaVel = thetaKP * thetaErr;

    double dist = (pose.topRows<2>() - target.topRows<2>()).norm();
    double xVel = dist <= 2 * doneThresh ? slowDriveVel : driveVel;

    return {.thetaVel = thetaVel, .xVel = xVel};
}

bool DriveToWaypointCommand::isDone() {
	double distance = (pose.topRows<2>() - target.topRows<2>()).norm();
	if (distance <= doneThresh) {
		if (!closeToTargetStartTime.has_value()) {
			closeToTargetStartTime = lastRecordedTime;
		}
		return lastRecordedTime.value() - closeToTargetStartTime.value() >= 
			   CLOSE_TO_TARGET_DUR;
	} else {
		closeToTargetStartTime.reset();
	}
	return false;
}

} // namespace commands
