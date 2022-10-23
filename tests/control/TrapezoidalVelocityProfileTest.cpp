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
