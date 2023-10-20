#include "../../src/control/PlanarArmController.h"

#include "../../src/Constants.h"

#include <iostream>

#include <catch2/catch.hpp>

using namespace Catch::literals;
using namespace control;
using namespace std::chrono_literals;

constexpr double PI = M_PI;

std::string toString(const Eigen::Vector2d& pose) {
	std::stringstream ss;
	ss << "(" << pose(0) << ", " << pose(1) << ")";
	return ss.str();
}

void assertApprox(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2, double dist = 1e-5,
				  double angle = 1e-5) {
	Eigen::Vector2d diff = p1 - p2;
	bool distEqual = diff.norm() <= dist;

	std::stringstream ss;
	ss << "Expected: " << toString(p1) << ", Actual: " << toString(p2);
	INFO(ss.str());

	REQUIRE(distEqual);
}

TEST_CASE("Test Planar Arm Controller", "[control][planararmcontroller]") {
	navtypes::Vectord<2> vec({0, 0});
	kinematics::PlanarArmKinematics<2> kin_obj(vec, vec, vec, 0.0, 0);
	PlanarArmController<2> foo({0, 0}, kin_obj, Constants::arm::SAFETY_FACTOR);
}

TEST_CASE("Test Planar Arm Safety Factor", "[control][planararmcontroller]") {
	// Set lengths and relative orientation bounds of robot joint segments.
	navtypes::Vectord<2> segLens({6.0, 4.0});
	navtypes::Vectord<2> minAngles({-M_PI, -M_PI});
	navtypes::Vectord<2> maxAngles({M_PI, M_PI});
	kinematics::PlanarArmKinematics<2> kin_obj(segLens, minAngles, maxAngles, 0.0, 0);

	// Instantiate PlanarArmController.
	PlanarArmController<2> foo({0, 0}, kin_obj, Constants::arm::SAFETY_FACTOR);

	// Attempt to straighten out end-effector all the way, exceeding max length.
	// This should cause the EE to be repositioned to fit the length constraint.
	foo.set_setpoint({0.0, 0.0});
	assertApprox({9.5, 0.0}, foo.get_setpoint(robot::types::dataclock::now()));

	// Try setting the joints to be orthogonal but within max length, such that:
	// - End effector position is (6,4), which implies that:
	// - Max length = sqrt(6^2 + 4^2) = sqrt(36 + 16) < 7.22 < 9.5 (max length)
	foo.set_setpoint({0.0, M_PI_2});
	assertApprox({6.0, 4.0}, foo.get_setpoint(robot::types::dataclock::now()));
}