#include "../../src/utils/core.h"

#include <catch2/catch.hpp>

using namespace util;

TEST_CASE("Test RAIIHelper", "[util][core]") {
	SECTION("Test RAIIHelper executes") {
		bool called = false;
		{
			RAIIHelper r([&]() { called = true; });
		}
		REQUIRE(called);
	}

	SECTION("Test RAIIHelper moves properly") {
		int i = 0;
		{
			RAIIHelper r([&]() { i++; });
			{ RAIIHelper r2(std::move(r)); }
			REQUIRE(i == 1);
		}
		REQUIRE(i == 1);
	}
}
