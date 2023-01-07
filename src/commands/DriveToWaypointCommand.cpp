#include "DriveToWaypointCommand.h"

using namespace navtypes;
using namespace robot::types;
using namespace std::chrono_literals;

static std::chrono::duration CLOSE_TO_TARGET_DUR = 1s;

namespace commands {

DriveToWaypointCommand::DriveToWaypointCommand(const point_t& target, double thetaKP,
											   double driveVel, double slowDriveVel,
											   double doneThresh)
	: target(target), thetaKP(thetaKP), driveVel(driveVel), slowDriveVel(slowDriveVel),
	  doneThresh(doneThresh), pose(pose_t::Zero()) {}

command_t DriveToWaypointCommand::getOutput() {
	Eigen::Vector2d toTarget = target.topRows<2>() - pose.topRows<2>();
	double targetAngle = std::atan2(toTarget(1), toTarget(0));
	double thetaErr =
		std::atan2(std::sin(targetAngle - pose(2)), std::cos(targetAngle - pose(2)));

	double thetaVel = thetaKP * thetaErr;

    double dist = (pose.topRows<2>() - target.topRows<2>()).norm();
    double xVel = dist <= 2 * doneThresh ? slowDriveVel : driveVel;

    return {.thetaVel = thetaVel, .xVel = xVel};
}

bool DriveToWaypointCommand::isDone() {
	if ((pose.topRows<2>() - target.topRows<2>()).norm() <= doneThresh) {
		if (!closeToTargetStartTime.has_value()) {
			datatime_t time = dataclock::now();
			closeToTargetStartTime = time;
		}
		return dataclock::now() - closeToTargetStartTime.value() >= CLOSE_TO_TARGET_DUR;
	} else {
		closeToTargetStartTime.reset();
	}
	return false;
}

} // namespace commands
