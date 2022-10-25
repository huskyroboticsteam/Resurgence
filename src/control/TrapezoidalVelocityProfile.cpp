#include "TrapezoidalVelocityProfile.h"

using navtypes::Vectord;

namespace control {

SingleDimTrapezoidalVelocityProfile::SingleDimTrapezoidalVelocityProfile(double maxVel,
																		 double maxAccel)
	: profile(maxVel, maxAccel) {}

void SingleDimTrapezoidalVelocityProfile::reset() {
	profile.reset();
}

void SingleDimTrapezoidalVelocityProfile::setTarget(robot::types::datatime_t currTime, double startPos, double endPos) {
	Vectord<1> startVec {startPos};
	Vectord<1> endVec {endPos};
	profile.setTarget(currTime, startVec, endVec);
}

bool SingleDimTrapezoidalVelocityProfile::hasTarget() const {
	return profile.hasTarget();
}

double SingleDimTrapezoidalVelocityProfile::getCommand(robot::types::datatime_t currTime) const {
	return profile.getCommand(currTime)(0);
}

std::optional<util::dseconds> SingleDimTrapezoidalVelocityProfile::getTotalTime() const {
	return profile.getTotalTime();
}

}