#include <catch2/catch.hpp>
#include "../../src/utils/time.h"

#include <chrono>

using namespace Catch::literals;
using namespace util;
using namespace std::chrono_literals;

TEST_CASE("Test Duration To Seconds", "[util]") {
	auto dur = 1500ms;
	REQUIRE(durationToSec(dur) == 1.5_a);
}

// TODO: add tests for the other util methods
