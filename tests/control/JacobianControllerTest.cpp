#include <catch2/catch.hpp>

#include "../../src/control/JacobianController.h"
#include "../../src/control/JacobianVelController.h"

using namespace Catch::literals;
using namespace control;
using namespace robot::types;
using namespace std::chrono_literals;

namespace {

constexpr double seg1 = 1.0, seg2 = 1.0;

Eigen::Vector2d kinematics(const Eigen::Vector2d& joints) {
	double joint1 = joints(0);
	double joint2 = joints(0) + joints(1);
	return {std::cos(joint1) * seg1 + std::cos(joint2) * seg2,
			std::sin(joint1) * seg1 + std::sin(joint2) * seg2};
}

}

TEST_CASE("JacobianController", "[control][jacobiancontroller]") {
	JacobianController<2,2> jc(kinematics, {}, {2, 2}, {1, 1});
	datatime_t startTime(0s);

	SECTION("Test Possible Target") {
		Eigen::Vector2d startPos = {0, M_PI/2};
		jc.setTarget(startTime, startPos, {1.2, 1.2});
		double cosineSim;
		Eigen::Vector2d command = jc.getCommand(startTime+500ms, startPos, cosineSim);
		REQUIRE(cosineSim == 1_a);
		REQUIRE(command(0) > startPos(0));
		REQUIRE(command(1) < startPos(1));
	}

	SECTION("Test Impossible Target") {
		Eigen::Vector2d startPos = {0, 0};
		jc.setTarget(startTime, startPos, {1, 0});
		double cosineSim;
		Eigen::Vector2d command = jc.getCommand(startTime+500ms, startPos, cosineSim);
		REQUIRE(std::isnan(cosineSim));
		REQUIRE((command - startPos).cwiseAbs().maxCoeff() <= 1e-4);
	}

	SECTION("Test Reset and Has Target") {
		REQUIRE_FALSE(jc.hasTarget());
		jc.setTarget(startTime, {0, 0}, {2, 1});
		REQUIRE(jc.hasTarget());
		jc.reset();
		REQUIRE_FALSE(jc.hasTarget());
	}
}
