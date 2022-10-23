#include <catch2/catch.hpp>

#include "../../src/control/TrapezoidalVelocityProfile.h"

using namespace Catch::literals;
using namespace control;
using namespace robot::types;
using namespace std::chrono;
using namespace std::chrono_literals;

TEST_CASE("Single Dimensional Trapezoidal Velocity Profile", "[control][trapvel]") {
	double maxVel = 2, maxAccel = 1;
	SingleDimTrapezoidalVelocityProfile profile(maxVel, maxAccel);
	datatime_t start(0s);

	SECTION("Test Forward Points") {
		profile.setTarget(start, 1, 11);

		REQUIRE(profile.getCommand(start + 0s) == 1_a);
		REQUIRE(profile.getCommand(start + 1s) == 1.5_a);
		REQUIRE(profile.getCommand(start + 2s) == 3.0_a);
		REQUIRE(profile.getCommand(start + 7s) == 11_a);
		REQUIRE(profile.getCommand(start + 6s) < 11);
		REQUIRE(profile.getCommand(start + 6s) == 10.5_a);
		REQUIRE(profile.getCommand(start + 5s) == 9_a);
		REQUIRE(profile.getCommand(start + 3500ms) == 6_a);
	}

	SECTION("Test Backward Points") {
		profile.setTarget(start, 11, 1);

		REQUIRE(profile.getCommand(start + 0s) == 11_a);
		REQUIRE(profile.getCommand(start + 1s) == 10.5_a);
		REQUIRE(profile.getCommand(start + 2s) == 9.0_a);
		REQUIRE(profile.getCommand(start + 7s) == 1_a);
		REQUIRE(profile.getCommand(start + 6s) > 1);
		REQUIRE(profile.getCommand(start + 6s) == 1.5_a);
		REQUIRE(profile.getCommand(start + 5s) == 3_a);
		REQUIRE(profile.getCommand(start + 3500ms) == 6_a);
	}

	SECTION("Test Reset") {
		REQUIRE(profile.getCommand(start + 3s) == 0);
		profile.setTarget(start, 1, 11);
		REQUIRE(profile.getCommand(start + 3s) != 0);
		profile.reset();
		REQUIRE(profile.getCommand(start + 3s) == 0);
	}

	SECTION("Test Has Target") {
		REQUIRE_FALSE(profile.hasTarget());
		profile.setTarget(start, 1, 2);
		REQUIRE(profile.hasTarget());
		profile.reset();
		REQUIRE_FALSE(profile.hasTarget());
	}

	SECTION("Test Monotonic") {
		int steps = 100;
		auto untilTime = start + 10s;

		profile.setTarget(start, 0, 100); // large enough that we never reach end
		for (datatime_t tPrev = start; tPrev < untilTime; tPrev += 10ms) {
			datatime_t tNext = tPrev + 10ms;
			REQUIRE(profile.getCommand(tPrev) < profile.getCommand(tNext));
		}

		profile.setTarget(start, 0, -100); // large enough that we never reach end
		for (datatime_t tPrev = start; tPrev < untilTime; tPrev += 10ms) {
			datatime_t tNext = tPrev + 10ms;
			REQUIRE(profile.getCommand(tPrev) > profile.getCommand(tNext));
		}
	}
}

TEST_CASE("Test Multi-Dimensional Trapezoidal Velocity Profile", "[control][trapvel]") {
	double maxVel = 2, maxAccel = 1;
	constexpr int dim = 2;
	TrapezoidalVelocityProfile<dim> profile({maxVel, maxVel}, {maxAccel, maxAccel});
	std::array<double, dim> startPos = {1, 11};
	std::array<double, dim> endPos = {11, 1};
	datatime_t startTime(0s);

	SECTION("Test Reset and Has Target") {
		REQUIRE_FALSE(profile.hasTarget());
		REQUIRE(profile.getCommand(startTime).array().isZero());
		profile.setTarget(startTime, startPos, endPos);
		REQUIRE(profile.hasTarget());
		REQUIRE_FALSE(profile.getCommand(startTime).array().isZero());
		profile.reset();
		REQUIRE_FALSE(profile.hasTarget());
		REQUIRE(profile.getCommand(startTime).array().isZero());
	}

	SECTION("Test Points") {
		profile.setTarget(startTime, startPos, endPos);

		REQUIRE(profile.getCommand(startTime + 0s).isApprox(Eigen::Vector2d{1, 11}));
		REQUIRE(profile.getCommand(startTime + 1s).isApprox(Eigen::Vector2d{1.5, 10.5}));
		REQUIRE(profile.getCommand(startTime + 2s).isApprox(Eigen::Vector2d{3, 9}));
		REQUIRE(profile.getCommand(startTime + 6s).isApprox(Eigen::Vector2d{10.5, 1.5}));
		REQUIRE(profile.getCommand(startTime + 7s).isApprox(Eigen::Vector2d{11, 1}));
	}

	SECTION("Test Set Target - Array") {
		profile.setTarget(startTime, startPos, endPos);
		REQUIRE(profile.getCommand(startTime + 0s).isApprox(Eigen::Vector2d{1, 11}));
		REQUIRE(profile.getCommand(startTime + 7s).isApprox(Eigen::Vector2d{11, 1}));
	}

	SECTION("Test Set Target - Vector") {
		Eigen::Matrix<double, dim, 1> startPosVec, endPosVec;
		for (int i = 0; i < dim; i++) {
			startPosVec(i) = startPos[i];
			endPosVec(i) = endPos[i];
		}
		profile.setTarget(startTime, startPosVec, endPosVec);
		REQUIRE(profile.getCommand(startTime + 0s).isApprox(Eigen::Vector2d{1, 11}));
		REQUIRE(profile.getCommand(startTime + 7s).isApprox(Eigen::Vector2d{11, 1}));
	}
}
