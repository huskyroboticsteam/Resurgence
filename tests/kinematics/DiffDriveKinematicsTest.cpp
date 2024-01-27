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

	constexpr double max_wheel_vel = 0.75;

	// Proportional Preservation Tests
	assertApprox({0.5, 0, 0.5},
				 kinematics.ensureWithinWheelSpeedLimit(
					 DiffDriveKinematics::PreferredVelPreservation::Proportional, 1.0, 1.0,
					 max_wheel_vel));

	// xVel Preservation Tests
	// Test #1: Test the xVel alone being too fast, therefore xVel should become the max and
	// theta should be zeroed
	assertApprox({-max_wheel_vel, 0, 0},
				 kinematics.ensureWithinWheelSpeedLimit(
					 DiffDriveKinematics::PreferredVelPreservation::PreferXVel, -1.0, 0.5,
					 max_wheel_vel));
	// Test #2: lVel < -max_wheel_vel
	assertApprox({-0.4, 0, 0.7}, kinematics.ensureWithinWheelSpeedLimit(
									 DiffDriveKinematics::PreferredVelPreservation::PreferXVel,
									 -0.4, 1.6, max_wheel_vel));
	// Test #3: lVel > max_wheel_vel: lVel = 1.0 rVel = 0.2
	assertApprox({0.6, 0, -0.3}, kinematics.ensureWithinWheelSpeedLimit(
									 DiffDriveKinematics::PreferredVelPreservation::PreferXVel,
									 0.6, -0.5714, max_wheel_vel));
	// Test #4: rVel < -max_wheel_vel: lVel = 0.2 rVel = -0.9
	assertApprox({-0.35, 0, -0.8},
				 kinematics.ensureWithinWheelSpeedLimit(
					 DiffDriveKinematics::PreferredVelPreservation::PreferXVel, -0.35, -1.1,
					 max_wheel_vel));
	// Test #5: rVel > max_wheel_vel: lVel = 0.2 rVel = 0.9
	assertApprox({0.55, 0, 0.4}, kinematics.ensureWithinWheelSpeedLimit(
									 DiffDriveKinematics::PreferredVelPreservation::PreferXVel,
									 0.55, 0.7, max_wheel_vel));

	// thetaVel Preservation Tests
	// Test #1: std::abs(wheelBaseWidth / 2 * thetaVel) > maxWheelSpeed - ie. thetaVel too big
	// by itself
	assertApprox({0, 0, 1.5},
				 kinematics.ensureWithinWheelSpeedLimit(
					 DiffDriveKinematics::PreferredVelPreservation::PreferThetaVel, 0, 1.6,
					 max_wheel_vel));
	// Test #2: std::max(lVel, rVel) > maxWheelSpeed: lVel = 0.9 r_vel = 0.35
	assertApprox({0.475, 0, -0.55},
				 kinematics.ensureWithinWheelSpeedLimit(
					 DiffDriveKinematics::PreferredVelPreservation::PreferThetaVel, 0.625,
					 -0.55, max_wheel_vel));
	// Test #3: std::min(lVel, rVel) < -maxWheelSpeed: lVel = -0.9 rVel = -0.2
	assertApprox({-0.4, 0, 0.7},
				 kinematics.ensureWithinWheelSpeedLimit(
					 DiffDriveKinematics::PreferredVelPreservation::PreferThetaVel, -0.55, 0.7,
					 max_wheel_vel));
}
