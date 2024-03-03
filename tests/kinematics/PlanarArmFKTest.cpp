#include "../../src/kinematics/PlanarArmFK.h"

#include <iostream>

#include <catch2/catch.hpp>

using namespace kinematics;

namespace {

void assertApprox(const Eigen::VectorXd& m1, const Eigen::VectorXd& m2, double eps = 1e-6) {
	INFO("m1=\n" << m1 << "\n\nm2=\n" << m2);
	REQUIRE((m1 - m2).array().abs().maxCoeff() < eps);
}

constexpr float PI = M_PI;
} // namespace

TEST_CASE("Test PlanarArmFK jointPosToEEPos", "[kinematics][planararm][fk]") {
	PlanarArmFK<2> kinematics({0.5, 0.5}, {-PI, -PI}, {PI, PI});

	assertApprox(kinematics.jointPosToEEPos({0, 0}), Eigen::Vector2d{1, 0});
	assertApprox(kinematics.jointPosToEEPos({PI, 0}), Eigen::Vector2d{-1, 0});
	assertApprox(kinematics.jointPosToEEPos({PI, PI}), Eigen::Vector2d{0, 0});
	assertApprox(kinematics.jointPosToEEPos({PI / 2, PI / 2}), Eigen::Vector2d{-0.5, 0.5});
	assertApprox(kinematics.jointPosToEEPos({-PI / 2, PI / 2}), Eigen::Vector2d{0.5, -0.5});
}

TEST_CASE("Test PlanarArmFK jointVelToEEVel", "[kinematics][planararm][fk]") {
	PlanarArmFK<2> kinematics({0.5, 0.5}, {-PI, -PI}, {PI, PI});

	assertApprox(kinematics.jointVelToEEVel({0, 0}, {0, 0}), Eigen::Vector2d{0, 0});
	assertApprox(kinematics.jointVelToEEVel({0, 0}, {1, 0}), Eigen::Vector2d{0, 1});
	assertApprox(kinematics.jointVelToEEVel({0, 0}, {0, 1}), Eigen::Vector2d{0, 0.5});
	assertApprox(kinematics.jointVelToEEVel({PI / 2, PI / 2}, {1, -1}),
				 Eigen::Vector2d{-0.5, 0});
}
