#include <catch2/catch.hpp>

#include "../../src/control/TrapezoidalVelocityProfile.h"

using namespace Catch::literals;
using namespace control;

TEST_CASE("Single Dimensional Trapezoidal Velocity Profile", "[control][trapvel]") {
	double maxVel = 2, maxAccel = 1;
	SingleDimTrapezoidalVelocityProfile profile(maxVel, maxAccel);

	SECTION("Test Forward Points") {
		profile.setTarget(1, 11);

		REQUIRE(profile.getCommand(0) == 1_a);
		REQUIRE(profile.getCommand(1) == 1.5_a);
		REQUIRE(profile.getCommand(2) == 3.0_a);
		REQUIRE(profile.getCommand(7) == 11_a);
		REQUIRE(profile.getCommand(6) < 11);
		REQUIRE(profile.getCommand(6) == 10.5_a);
		REQUIRE(profile.getCommand(5) == 9_a);
		REQUIRE(profile.getCommand(3.5) == 6_a);
	}

	SECTION("Test Backward Points") {
		profile.setTarget(11, 1);

		REQUIRE(profile.getCommand(0) == 11_a);
		REQUIRE(profile.getCommand(1) == 10.5_a);
		REQUIRE(profile.getCommand(2) == 9.0_a);
		REQUIRE(profile.getCommand(7) == 1_a);
		REQUIRE(profile.getCommand(6) > 1);
		REQUIRE(profile.getCommand(6) == 1.5_a);
		REQUIRE(profile.getCommand(5) == 3_a);
		REQUIRE(profile.getCommand(3.5) == 6_a);
	}

	SECTION("Test Reset") {
		REQUIRE(profile.getCommand(3) == 0);
		profile.setTarget(1, 11);
		REQUIRE(profile.getCommand(3) != 0);
		profile.reset();
		REQUIRE(profile.getCommand(3) == 0);
	}

	SECTION("Test Monotonic") {
		int steps = 100;

		profile.setTarget(0, 100); // large enough that we never reach end
		for (int i = 0; i < 10 * steps; i++) {
			double tPrev = static_cast<double>(i) / steps;
			double tNext = static_cast<double>(i+1) / steps;
			REQUIRE(profile.getCommand(tPrev) < profile.getCommand(tNext));
		}

		profile.setTarget(0, -100); // large enough that we never reach end
		for (int i = 0; i < 10 * steps; i++) {
			double tPrev = static_cast<double>(i) / steps;
			double tNext = static_cast<double>(i+1) / steps;
			REQUIRE(profile.getCommand(tPrev) > profile.getCommand(tNext));
		}
	}
}
