#include "../../src/kinematics/SwerveDriveKinematics.h"

#include "../../src/navtypes.h"

#include <cmath>
#include <iostream>

#include <catch2/catch.hpp>

using namespace navtypes;
using namespace kinematics;

namespace {
constexpr double PI = M_PI;

std::string toString(const pose_t& pose) {
	std::stringstream ss;
	ss << "(" << pose(0) << ", " << pose(1) << ", " << pose(2) << ")";
	return ss.str();
}

void assertApprox(const pose_t& p1, const pose_t& p2, double dist = 1e-5,
				  double angle = 1e-5) {
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
	return {lfVel, lfRot, rfVel, rfRot, lbVel, lbRot, rbVel, rbRot};
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

	// drive forward 8 m, left 10 m, rotate 0.84 * PI radians CCW w/ same velocity of previous test
	assertApprox({8, 10, 0.84 * PI}, kinematics.getPoseUpdate(targetWheelVels, 0, 2));

	// drive forward 3 m, left 7 m, rotate 0.25 * PI radians (45 degrees) CCW from existing 45-degree CCW heading
	// expected xf = -4/sqrt(2) = -2.828427125 , yf = 10/sqrt(2) = 7.071067812, angle = 0.5*PI
	targetWheelVels = kinematics.robotVelToWheelVel(3, 7, 0.25 * PI);
	assertApprox({-2.828427125, 7.071067812, 0.5 * PI},
				  kinematics.getNextPose(targetWheelVels, {0, 0, 0.25 * PI}, 1));

	// drive forward 6 m, left 14 m, rotate 0.5 * PI radians (90 degrees) CCW from existing 45-degree CCW heading
	// w/ same velocity of previous test
	// expected xf = -8/sqrt(2), yf = 20/sqrt(2), angle = 0.75*PI
	assertApprox({-2.828427125 * 2, 7.071067812 * 2, 0.75 * PI},
				  kinematics.getNextPose(targetWheelVels, {0, 0, 0.25 * PI}, 2));

	// drive forward 6 m, left 14 m, rotate 0.5 * PI radians (90 degrees) CCW from existing 45-degree CCW heading
	// w/ same velocity of previous test from starting position 3 meters forward, 4 meters left
	// expected xf = 3 - 8/sqrt(2), yf = 4 + 20/sqrt(2), angle = 0.75*PI
	assertApprox({3.0 - (2.828427125 * 2), 4.0 + (7.071067812 * 2), 0.75 * PI},
				  kinematics.getNextPose(targetWheelVels, {3, 4, 0.25 * PI}, 2));
}