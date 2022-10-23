#include "TrapezoidalVelocityProfile.h"
#include <chrono>

using namespace control;
using std::chrono::duration_cast;
using std::chrono::milliseconds;

SingleDimTrapezoidalVelocityProfile::SingleDimTrapezoidalVelocityProfile(double maxVel,
																		 double maxAccel)
	: maxVel(maxVel), maxAccel(maxAccel) {}

void SingleDimTrapezoidalVelocityProfile::reset() {
	profile.reset();
}

void SingleDimTrapezoidalVelocityProfile::setTarget(robot::types::datatime_t currTime, double startPos, double endPos) {
	profile_t profile{.startTime = currTime, .startPos = startPos, .endPos = endPos};
	double rampUpTime = maxVel / maxAccel;
	double rampUpDist = std::pow(maxVel, 2) / (2 * maxAccel);

	double dist = std::abs(endPos - startPos);
	if (2 * rampUpDist < dist) {
		profile.stopAccelTime = rampUpTime;
		double coastTime = (dist - 2 * rampUpDist) / maxVel;
		profile.startDecelTime = rampUpTime + coastTime;
		profile.totalTime = coastTime + 2 * rampUpTime;
	} else {
		double totalTime = 2 * std::sqrt(dist / maxAccel);
		profile.stopAccelTime = profile.startDecelTime = totalTime / 2;
		profile.totalTime = totalTime;
	}
	this->profile = profile;
}

bool SingleDimTrapezoidalVelocityProfile::hasTarget() const {
	return this->profile.has_value();
}

double SingleDimTrapezoidalVelocityProfile::getCommand(robot::types::datatime_t currTime) const {
	if (!profile) {
		return 0;
	}

	profile_t profile = this->profile.value();
	double sign = profile.endPos - profile.startPos;
	double elapsedTime = duration_cast<milliseconds>(currTime - profile.startTime).count() / 1000.0;
	if (elapsedTime < 0) {
		return profile.startPos;
	} else if (elapsedTime < profile.stopAccelTime) {
		double dist = 0.5 * maxAccel * std::pow(elapsedTime, 2);
		return profile.startPos + std::copysign(dist, sign);
	} else if (elapsedTime < profile.startDecelTime) {
		// area of trapezoid is average of bases times height
		double dist = (2 * elapsedTime - profile.stopAccelTime) / 2.0 * maxVel;
		return profile.startPos + std::copysign(dist, sign);
	} else if (elapsedTime < profile.totalTime) {
		double distFromEnd = 0.5 * maxAccel * std::pow(profile.totalTime - elapsedTime, 2);
		return profile.endPos - std::copysign(distFromEnd, sign);
	} else {
		return profile.endPos;
	}
}