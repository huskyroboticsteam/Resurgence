#include "../Constants.h"
#include "../Globals.h"
#include "../Util.h"
#include "../kinematics/DiffDriveKinematics.h"
#include "../navtypes.h"
#include "world_interface.h"

#include <chrono>

using namespace navtypes;
using namespace robot::types;
using util::toTransform;

using std::chrono::duration_cast;
using std::chrono::milliseconds;

static DataPoint<transform_t> lastOdom;
static wheelvel_t commandedWheelVel{0, 0};

namespace robot {
namespace {
void setCmdVelToIntegrate(const wheelvel_t& wheelVels) {
	auto odom = robot::readOdom();
	lastOdom = odom;
	commandedWheelVel = wheelVels;
}
} // namespace

DataPoint<transform_t> readOdom() {
	if (!lastOdom) {
		return toTransform({0, 0, 0});
	} else {
		datatime_t now = dataclock::now();
		double elapsed =
			duration_cast<milliseconds>(now - lastOdom.getTime()).count() / 1000.0;
		transform_t update =
			toTransform(driveKinematics().getLocalPoseUpdate(commandedWheelVel, elapsed));
		transform_t odom = lastOdom.getData() * update;
		lastOdom = {now, odom};
		return lastOdom;
	}
}

double setCmdVel(double dtheta, double dx) {
	if (Globals::E_STOP && (dtheta != 0 || dx != 0)) {
		return 0;
	}

	wheelvel_t wheelVels = driveKinematics().robotVelToWheelVel(dx, dtheta);
	double lPWM = wheelVels.lVel / Constants::MAX_WHEEL_VEL;
	double rPWM = wheelVels.rVel / Constants::MAX_WHEEL_VEL;
	double maxAbsPWM = std::max(std::abs(lPWM), std::abs(rPWM));
	if (maxAbsPWM > 1) {
		lPWM /= maxAbsPWM;
		rPWM /= maxAbsPWM;
	}

	setCmdVelToIntegrate(wheelVels);
	setMotorPower(motorid_t::frontLeftWheel, lPWM);
	setMotorPower(motorid_t::rearLeftWheel, lPWM);
	setMotorPower(motorid_t::frontRightWheel, rPWM);
	setMotorPower(motorid_t::rearRightWheel, rPWM);

	return maxAbsPWM > 1 ? maxAbsPWM : 1.0;
}

std::pair<double, double> getCmdVel() {
	double l = commandedWheelVel.lVel;
	double r = commandedWheelVel.rVel;
	pose_t robotVel = driveKinematics().wheelVelToRobotVel(l, r);
	return {robotVel(2), robotVel(0)};
}

} // namespace robot
