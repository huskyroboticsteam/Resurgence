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

	SECTION("Test Total Time") {
		profile.setTarget(start, 1, 11);
		REQUIRE(profile.getTotalTime()->count() == 7_a);
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
	TrapezoidalVelocityProfile<dim> profile(maxVel, maxAccel);
	Eigen::Vector2d startPos = {1, 11};
	Eigen::Vector2d endPos = {11, 1};
	datatime_t startTime(0s);

	SECTION("Test Reset and Has Target") {
		REQUIRE_FALSE(profile.hasTarget());
		REQUIRE(profile.getCommand(startTime).array().isZero());
		REQUIRE_FALSE(profile.getTotalTime().has_value());

		profile.setTarget(startTime, startPos, endPos);

		REQUIRE(profile.hasTarget());
		REQUIRE_FALSE(profile.getCommand(startTime).array().isZero());
		REQUIRE(profile.getTotalTime().has_value());

		profile.reset();

		REQUIRE_FALSE(profile.hasTarget());
		REQUIRE(profile.getCommand(startTime).array().isZero());
		REQUIRE_FALSE(profile.getTotalTime().has_value());
	}

	SECTION("Test Total Time") {
		profile.setTarget(startTime, startPos, endPos);
		REQUIRE(profile.getTotalTime()->count() == 9.07107_a);

		Eigen::Vector2d start = {0, 0};
		Eigen::Vector2d end = {10, 4};
		profile.setTarget(startTime, start, end);
		REQUIRE(profile.getTotalTime()->count() == 7.38516_a);
	}

	SECTION("Test Points") {
		profile.setTarget(startTime, startPos, endPos);
		auto profileTime = duration_cast<nanoseconds>(profile.getTotalTime().value());

		REQUIRE(profile.getCommand(startTime + 0s).isApprox(startPos));
		REQUIRE(profile.getCommand(startTime + 2s).isApprox(startPos + (endPos - startPos).normalized() * 2));
		REQUIRE(profile.getCommand(startTime + profileTime / 2).isApprox((startPos + endPos)/2, 1e-5));
		REQUIRE(profile.getCommand(startTime + profileTime - 2s).isApprox(endPos + (startPos - endPos).normalized() * 2, 1e-5));
		REQUIRE(profile.getCommand(startTime + profileTime).isApprox(endPos));
	}

	SECTION("Test Before and After Timestamps") {
		profile.setTarget(startTime, startPos, endPos);
		auto profileTime = duration_cast<nanoseconds>(profile.getTotalTime().value());

		REQUIRE(profile.getCommand(startTime - 1s).isApprox(startPos));
		REQUIRE(profile.getCommand(startTime + profileTime + 1s).isApprox(endPos));
	}

	SECTION("Test Set Target - Vector") {
		profile.setTarget(startTime, startPos, endPos);
		REQUIRE(profile.getCommand(startTime + 0s).isApprox(Eigen::Vector2d{1, 11}));
		REQUIRE(profile.getCommand(startTime + duration_cast<nanoseconds>(profile.getTotalTime().value()))
				.isApprox(Eigen::Vector2d{11, 1}));
	}

	SECTION("Test Set Target - Vector") {
		std::array<double, dim> startPosArr, endPosArr;
		for (int i = 0; i < dim; i++) {
			startPosArr[i] = startPos(i);
			endPosArr[i] = endPos(i);
		}
		profile.setTarget(startTime, startPosArr, endPosArr);
		REQUIRE(profile.getCommand(startTime + 0s).isApprox(Eigen::Vector2d{1, 11}));
		REQUIRE(profile.getCommand(startTime + duration_cast<nanoseconds>(profile.getTotalTime().value()))
				.isApprox(Eigen::Vector2d{endPos[0], endPos[1]}));
	}
}
