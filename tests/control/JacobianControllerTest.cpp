#include "../../src/control/JacobianController.h"

#include "../../src/control/JacobianVelController.h"

#include <catch2/catch.hpp>

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

} // namespace

TEST_CASE("JacobianController", "[control][jacobiancontroller]") {
	JacobianController<2, 2> jc(kinematics, {}, 2, 1);
	datatime_t startTime(0s);

	SECTION("Test Possible Target") {
		// test that going to a possible target will have a correct target (cosine sim=1)
		Eigen::Vector2d startPos = {0, M_PI / 2};
		jc.setTarget(startTime, startPos, {1.2, 1.2});
		double cosineSim;
		Eigen::Vector2d command = jc.getCommand(startTime + 500ms, startPos, cosineSim);
		REQUIRE(cosineSim == 1_a);
		REQUIRE(command(0) > startPos(0));
		REQUIRE(command(1) < startPos(1));
	}

	SECTION("Test Impossible Target") {
		// test that going to an impossible target will have a non-one cosine similarity
		Eigen::Vector2d startPos = {0, 0};
		jc.setTarget(startTime, startPos, {1, 0});
		double cosineSim;
		Eigen::Vector2d command = jc.getCommand(startTime + 500ms, startPos, cosineSim);
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

TEST_CASE("JacobianVelController", "[control][jacobiancontroller]") {
	JacobianVelController<2, 2> jvc(kinematics, {});
	datatime_t startTime(1s);

	SECTION("Test Reset and Has Target") {
		Eigen::Vector2d pos = {1, 1};
		REQUIRE_FALSE(jvc.hasTarget());
		jvc.setTarget(startTime, {0, 0});
		REQUIRE(jvc.hasTarget());
		jvc.reset();
		REQUIRE_FALSE(jvc.hasTarget());
	}

	SECTION("Test Possible Target") {
		// test that going to a possible target will have a correct target (cosine sim=1)
		Eigen::Vector2d startPos = {0, M_PI / 2};
		jvc.setTarget(startTime, {1.0, 1.0});
		double cosineSim;
		Eigen::Vector2d command = jvc.getCommand(startTime + 500ms, startPos, cosineSim);
		REQUIRE(cosineSim == 1_a);
		REQUIRE(command(0) > startPos(0));
		REQUIRE(command(1) < startPos(1));
	}

	SECTION("Test Impossible Target") {
		// test that going to an impossible target will have a non-one cosine similarity
		Eigen::Vector2d startPos = {0, 0};
		jvc.setTarget(startTime, {-1, 0});
		double cosineSim;
		Eigen::Vector2d command = jvc.getCommand(startTime + 500ms, startPos, cosineSim);
		REQUIRE(std::isnan(cosineSim));
		REQUIRE((command - startPos).cwiseAbs().maxCoeff() <= 1e-4);
	}
}
