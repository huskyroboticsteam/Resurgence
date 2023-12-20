#include "../../src/kinematics/DiffDriveKinematics.h"

#include "../../src/navtypes.h"

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

wheelvel_t wheelVel(double lVel, double rVel) {
	return (wheelvel_t){lVel, rVel};
}
} // namespace

TEST_CASE("Differential Drive Kinematics Test") {
	DiffDriveKinematics kinematics(1);

	// go straight forward 1m
	assertApprox({1, 0, 0}, kinematics.getPoseUpdate(wheelVel(1, 1), 0, 1));
	// turn 90deg CCW in place
	assertApprox({0, 0, PI / 2}, kinematics.getPoseUpdate(wheelVel(-1, 1), 0, PI / 4));
	// start at (0, -1) facing forward, drive forward while turning 90deg CCW
	assertApprox({1, 0, PI / 2},
				 kinematics.getNextPose(wheelVel(PI / 4, 3 * PI / 4), {0, -1, 0}, 1));
	// drive CW in a full circle with radius 1
	assertApprox({0, 0, 0},
				 kinematics.getNextPose(wheelVel(3 * PI / 4, PI / 4), {0, 0, 0}, 4));

	double max_wheel_vel = 0.75;

	// Proportional Preservation Tests
	assertApprox({0.5, 0, 0.5},
				 kinematics.ensureWithinWheelSpeedLimit(
					 kinematics::DiffDriveKinematics::PreferredVelPreservation::Proportional,
					 1.0, 1.0, max_wheel_vel));

	// xVel Preservation Tests
	assertApprox({-max_wheel_vel, 0, 0},
				 kinematics.ensureWithinWheelSpeedLimit(
					 kinematics::DiffDriveKinematics::PreferredVelPreservation::PreferXVel,
					 -1.0, 0.5, max_wheel_vel));
	assertApprox({0.5, 0, 0.5},
				 kinematics.ensureWithinWheelSpeedLimit(
					 kinematics::DiffDriveKinematics::PreferredVelPreservation::PreferXVel,
					 0.5, 0.5, max_wheel_vel));
	assertApprox({0, 0, 0},
				 kinematics.ensureWithinWheelSpeedLimit(
					 kinematics::DiffDriveKinematics::PreferredVelPreservation::PreferXVel,
					 0.5, 0.5, max_wheel_vel));

	// thetaVel Preservation Tests
	assertApprox({0, 0, 0},
				 kinematics.ensureWithinWheelSpeedLimit(
					 kinematics::DiffDriveKinematics::PreferredVelPreservation::PreferThetaVel,
					 0.5, 0.5, max_wheel_vel));
	assertApprox({0, 0, 0},
				 kinematics.ensureWithinWheelSpeedLimit(
					 kinematics::DiffDriveKinematics::PreferredVelPreservation::PreferThetaVel,
					 0.5, 0.5, max_wheel_vel));
}
