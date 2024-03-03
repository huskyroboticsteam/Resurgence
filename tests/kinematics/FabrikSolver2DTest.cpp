#include "../../src/kinematics/FabrikSolver.h"
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

TEST_CASE("Test FabrikSolver2D eePosToJointPos", "[kinematics][planararm][ik][fabrik]") {
	double thresh = 0.001;
	FabrikSolver2D<2> ik({0.5, 0.5}, {-PI, -PI}, {PI, PI}, thresh, 50);

	SECTION("Test out of range targets") {
		{
			bool success;
			Eigen::Vector2d jp = ik.eePosToJointPos({3, 0}, {0.1, 0.1}, success);
			REQUIRE_FALSE(success);
			assertApprox(jp, Eigen::Vector2d{0.1, 0.1});
		}
		{
			bool success;
			Eigen::Vector2d jp = ik.eePosToJointPos({-2, 1}, {0, PI}, success);
			REQUIRE_FALSE(success);
			assertApprox(jp, Eigen::Vector2d{0, PI});
		}
	}

	SECTION("Test in range targets") {
		PlanarArmFK<2> fk({0.5, 0.5}, {-PI, -PI}, {PI, PI});
		for (int i = 0; i < 100; i++) {
			Eigen::Vector2d target;
			target.setRandom();
			if (target.norm() > 0.9) {
				target = 0.9 * target.normalized().eval();
			}
			bool success;
			Eigen::Vector2d jp = ik.eePosToJointPos(target, {0.0, 0.0}, success);
			REQUIRE(success);
			assertApprox(fk.jointPosToEEPos(jp), target, thresh);
		}
	}

	SECTION("Test joint constraints") {
		FabrikSolver2D<2> constrained({0.5, 0.5}, {-PI / 8, -PI}, {PI / 8, PI}, thresh, 50);
		Eigen::Vector2d target = {std::sqrt(2) / 2, 0};
		bool success;
		Eigen::Vector2d jp = constrained.eePosToJointPos(target, {PI / 2, PI / 2}, success);
		INFO("jp:\n" << jp);
		REQUIRE_FALSE(success);
		assertApprox(jp, Eigen::Vector2d{PI / 2, PI / 2});
	}
}
