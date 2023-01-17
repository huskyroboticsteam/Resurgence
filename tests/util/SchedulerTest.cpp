#include "../../src/utils/scheduler.h"

#include <chrono>
#include <thread>

#include <catch2/catch.hpp>

using namespace util;
using namespace std::chrono_literals;

TEST_CASE("Test PeriodicScheduler", "[util]") {
	PeriodicScheduler sched;
	int i = 0;
	int j = 0;
	sched.scheduleEvent(100ms, [&]() { i += 1; });
	sched.scheduleEvent(75ms, [&]() { j += 1; });
	std::this_thread::sleep_for(250ms);
	REQUIRE(i == 2);
	REQUIRE(j == 3);
	sched.clear();
	std::this_thread::sleep_for(250ms);
	REQUIRE(i == 2);
	REQUIRE(j == 3);
	sched.scheduleEvent(100ms, [&]() { i += 1; });
	std::this_thread::sleep_for(150ms);
	REQUIRE(i == 3);
}
