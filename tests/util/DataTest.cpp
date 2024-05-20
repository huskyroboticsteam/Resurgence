#include "../../src/world_interface/data.h"

#include <catch2/catch.hpp>

using namespace robot::types;
using namespace robot;
using namespace std::chrono_literals;

TEST_CASE("Test DataPoint", "[util][data]") {
	// test empty datapoint
	DataPoint<int> empty;
	REQUIRE_THROWS_AS(empty.getData(), bad_datapoint_access);
	REQUIRE_FALSE(empty.isValid());
	REQUIRE_FALSE(static_cast<bool>(empty));
	REQUIRE_FALSE(empty.isFresh(100ms));

	// test nonempty datapoint
	auto now = dataclock::now();
	DataPoint<int> dp(now, 42);
	REQUIRE(dp.getTime() == now);
	REQUIRE(dp.getData() == 42);
	REQUIRE(dp.isValid());
	REQUIRE(static_cast<bool>(dp));
	REQUIRE(dp.isFresh(100ms));
}
