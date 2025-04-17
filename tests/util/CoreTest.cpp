#include "../../src/utils/core.h"

#include <catch2/catch.hpp>

using namespace util;

TEST_CASE("Test almostEqual", "[util][core]") {
	SECTION("Numbers are approximately equal") {
		REQUIRE(almostEqual(1.000000001, 1.000000002));
		REQUIRE(almostEqual(1.0, 1.0, 1e-5));
	}

	SECTION("Numbers are not approximately equal") {
		REQUIRE_FALSE(almostEqual(1.0, 1.1));
		REQUIRE_FALSE(almostEqual(1.0, 1.0 + 1e-5, 1e-6));
	}
}

TEST_CASE("Test Key Set", "[util][core]") {
	SECTION("Extracting keys from a given map") {
		std::unordered_map<std::string, int> inputtedMap = {
			{"key1", 1}, {"key2", 2}, {"key3", 3}};

		auto keys = keySet(inputtedMap);

		std::unordered_set<std::string> expectedSet = {"key1", "key2", "key3"};

		REQUIRE(keys == expectedSet);
	}

	SECTION("Empty map results in empty set") {
		std::unordered_map<std::string, int> emptyMap;
		auto keys = keySet(emptyMap);
		REQUIRE(keys.empty());
	}
}

TEST_CASE("Test To String", "[util][core]") {
	SECTION("Convert integers to strings") {
		REQUIRE(to_string(69) == "69");
		REQUIRE(to_string(-1) == "-1");
	}

	SECTION("Convert floats to strings") {
		REQUIRE(to_string(3.14).substr(0, 4) == "3.14");
		REQUIRE(to_string(1e-6) == "0.000001");
	}

	SECTION("Convert booleans to strings") {
		REQUIRE(to_string(true) == "true");
		REQUIRE(to_string(false) == "false");
	}
}

TEST_CASE("Test Freeze String", "[util][core]") {
	SECTION("Converting std::string to frozen::string") {
		std::string stdString = "Test";
		auto frozenString = freezeStr(stdString);
		REQUIRE(frozenString == frozen::string("Test"));
	}

	SECTION("Converting empty std::string results in empty frozen::string") {
		std::string stdString;
		auto frozenString = freezeStr(stdString);
		REQUIRE(frozenString == frozen::string(""));
	}
}

TEST_CASE("Test Pair To Tuple", "[util][core]") {
	SECTION("Converting pair of integers to tuple") {
		std::pair<int, int> pair = {0, 1};
		auto tuple = pairToTuple(pair);
		REQUIRE(std::get<0>(tuple) == 0);
		REQUIRE(std::get<1>(tuple) == 1);
	}

	SECTION("Converting mixed pair of string and integer to tuple") {
		std::pair<std::string, int> pair = {"key0", 0};
		auto tuple = pairToTuple(pair);
		REQUIRE(std::get<0>(tuple) == "key0");
		REQUIRE(std::get<1>(tuple) == 0);
	}
}

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
