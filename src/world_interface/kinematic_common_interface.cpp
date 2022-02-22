#include "kinematic_common_interface.h"

#include "../Constants.h"
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

static DiffDriveKinematics kinematics(Constants::EFF_WHEEL_BASE);
static DataPoint<transform_t> lastOdom;
static wheelvel_t commandedWheelVel{0, 0};

namespace robot {

DataPoint<transform_t> readOdom() {
	if (!lastOdom) {
		return toTransform({0, 0, 0});
	} else {
		datatime_t now = dataclock::now();
		double elapsed =
			duration_cast<milliseconds>(now - lastOdom.getTime()).count() / 1000.0;
		transform_t update =
			toTransform(kinematics.getLocalPoseUpdate(commandedWheelVel, elapsed));
		transform_t odom = lastOdom.getData() * update;
		lastOdom = {now, odom};
		return lastOdom;
	}
}

std::pair<double, double> getCmdVel() {
	double l = commandedWheelVel.lVel;
	double r = commandedWheelVel.rVel;
	pose_t robotVel = kinematics.wheelVelToRobotVel(l, r);
	return {robotVel(2), robotVel(0)};
}

void setCmdVelToIntegrate(const wheelvel_t& wheelVels) {
	auto odom = robot::readOdom();
	lastOdom = odom;
	commandedWheelVel = wheelVels;
}

void setCmdVelToIntegrate(double dtheta, double dx) {
	setCmdVelToIntegrate(kinematics.robotVelToWheelVel(dx, dtheta));
}

} // namespace robot
