#include "../../src/utils/json.h"

#include <catch2/catch.hpp>

using namespace util;

TEST_CASE("Test Has Key", "[util][json]") {
	json j = {{"name", "huskyRobos"}, {"location", "University of Washington"}};

	REQUIRE(hasKey(j, "name"));			   // has key of 'name'
	REQUIRE_FALSE(hasKey(j, "numLosses")); // doesn't have key of 'numLosses'
}

TEST_CASE("Test Validate Key (Given Individual Types)", "[util][json]") {
	json j = {{"name", "huskyRobos"}, {"age", 11}, {"cool", true}};
	REQUIRE(validateKey(j, "name", val_t::string)); // has key of 'name' that is a string
	REQUIRE_FALSE(
		validateKey(j, "age", val_t::string)); // doesn't have key of 'age' that is a string
	REQUIRE(validateKey(j, "cool", val_t::boolean)); // has key of 'cool' that is a boolean
}

TEST_CASE("Test Validate Key (Given Set of Types)", "[util][json]") {
	json j = {{"name", "huskyRobos"}, {"age", 11}, {"cool", true}};

	std::unordered_set<val_t> keyTypesAllowed = {val_t::string, val_t::number_integer};

	REQUIRE(validateKey(
		j, "name", keyTypesAllowed)); // has key of 'name' that is a type of the allowed types
	REQUIRE(validateKey(
		j, "age", keyTypesAllowed)); // has key of 'age' that is a type of the allowed types
	REQUIRE_FALSE(validateKey(
		j, "cool",
		keyTypesAllowed)); // doesn't have key of 'cool' that is a type of the allowed types
}

TEST_CASE("Test Validate One Of", "[util][json]") {
	json j = {{"status", "active"}, {"subTeams", "software"}};

	std::unordered_set<std::string> statusValueTypesAllowed = {"active", "inactive"};
	std::unordered_set<std::string> subTeamValueTypesAllowed = {"software", "electrical",
																"mechanical"};

	REQUIRE(validateOneOf(j, "status",
						  statusValueTypesAllowed)); // value of 'status' is a string in set of
													 // statusValueTypesAllowed
	REQUIRE_FALSE(validateOneOf(
		j, "status", {"pending", "closed"})); // value of 'status' is a string but not in set
											  // of statusValueTypesAllowed
	REQUIRE(validateOneOf(j, "subTeams",
						  subTeamValueTypesAllowed)); // value of 'subTeams' is a string in set
													  // of subTeamValueTypesAllowed
}

TEST_CASE("Test Validate Range", "[util][json]") {
	json j = {
		{"winLossRatio", 0.8375},
		{"avgGPA", 3.6},
	};

	REQUIRE(validateRange(j, "winLossRatio", 0.0,
						  1.0)); // 'winLossRatio' is in range from 0.0 to 1.0
	REQUIRE(validateRange(j, "avgGPA", 0.0, 4.0)); // 'avgGPA' is in range from 0.0 to 4.0
	REQUIRE_FALSE(
		validateRange(j, "avgGPA", 0.0, 2.0)); // 'avgGPA' isnt in range of 0.0 to 2.0
}