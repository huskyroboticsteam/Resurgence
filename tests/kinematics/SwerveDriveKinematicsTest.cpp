#include "../../src/kinematics/SwerveDriveKinematics.h"

#include "../../src/navtypes.h"

#include <cmath>
#include <iostream>

#include <catch2/catch.hpp>

using namespace navtypes;

namespace {
constexpr double PI = M_PI;

std::string toString(const pose_t& pose) {
	std::stringstream ss;
	ss << "(" << pose(0) << ", " << pose(1) << ", " << pose(2) << ")";
	return ss.str();
}

void assertApprox(const pose_t& p1, const pose_t& p2, double dist = 0.01,
				  double angle = 0.01) {
	pose_t diff = p1 - p2;
	bool distEqual = diff.topRows<2>().norm() <= dist;
	double thetaDiff = std::fmod(abs(diff(2)), 2 * PI);
	// change domain from [0, 2pi) to (-pi, pi]
	if (thetaDiff > PI) {
		thetaDiff -= 2 * PI;
	}
	bool angleEqual = abs(thetaDiff) <= angle;
	if (distEqual && angleEqual) {
		SUCCEED();
	} else {
		std::cout << "Expected: " << toString(p1) << ", Actual: " << toString(p2) << std::endl;
		FAIL();
	}
}

swervewheelvel_t wheelVel(double lfVel, double lfRot, double rfVel, double rfRot, double lbVel,
						  double lbRot, double rbVel, double rbRot) {
	return (swervewheelvel_t){lfVel, lfRot, rfVel, rfRot, lbVel, lbRot, rbVel, rbRot};
}
} // namespace

TEST_CASE("Swerve Drive Kinematics Test") {
	SwerveDriveKinematics kinematics(1, 1);

	// go straight forward 1 m
	assertApprox({1, 0, 0}, kinematics.getPoseUpdate(wheelVel(1, 0, 1, 0, 1, 0, 1, 0), 0, 1));

	// go 3 m forward, 4 meters left
	assertApprox({3, 4, 0}, kinematics.getPoseUpdate(wheelVel(5, 0.927295218, 5, 0.927295218,
															  5, 0.927295218, 5, 0.927295218),
													 0, 1));

	// turn 90deg CCW in place
	double wv = sqrt(2) * PI / 4;
	double wr = PI / 4;
	assertApprox(
		{0, 0, PI / 2},
		kinematics.getPoseUpdate(wheelVel(wv, wr * 3, wv, wr, wv, wr * 5, wv, wr * 7), 0, 1));

	// drive forward 4 m, left 5 m, rotate 0.42 * PI radians CCW
	swervewheelvel_t targetWheelVels = kinematics.robotVelToWheelVel(4, 5, 0.42 * PI);
	assertApprox({4, 5, 0.42 * PI}, kinematics.getPoseUpdate(targetWheelVels, 0, 1));
}