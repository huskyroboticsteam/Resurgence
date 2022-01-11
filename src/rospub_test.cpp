#include "rospub.h"

#include <array>

#include <catch2/catch.hpp>

using json = nlohmann::json;

namespace rospub {

constexpr std::array<PointPub, 5> PointPub_values = {PLAN_VIZ, CURRENT_POSE, DRIVE_TARGET,
													 PLAN_TARGET, POSE_GRAPH};
constexpr std::array<ArrayPub, 2> ArrayPub_values = {LIDAR_SCAN, LANDMARKS};

TEST_CASE("PointPub enum serialized correctly") {
	// go through all possible PointPub values
	for (PointPub value : PointPub_values) {
		// add the value to a JSON object and serialize it.
		json j;
		j["value"] = value;
		std::string serialized;
		REQUIRE_NOTHROW(serialized = j.dump());

		// deserialize the json message, get the value, and check it.
		json deserialized;
		REQUIRE_NOTHROW(deserialized = json::parse(serialized));
		PointPub check = deserialized.at("value").get<PointPub>();
		REQUIRE(value == check);
	}
}

TEST_CASE("ArrayPub enum serialized correctly") {
	// go through all possible ArrayPub values
	for (ArrayPub value : ArrayPub_values) {
		// add the value to a JSON object and serialize it.
		json j;
		j["value"] = value;
		std::string serialized;
		REQUIRE_NOTHROW(serialized = j.dump());

		// deserialize the JSON message, get the value, and check it.
		json deserialized;
		REQUIRE_NOTHROW(deserialized = json::parse(serialized));
		ArrayPub check = deserialized.at("value").get<ArrayPub>();
		REQUIRE(value == check);
	}
}

TEST_CASE("ArrayPub cannot be deserialized to PointPub") {
	// go through all possible ArrayPub values
	for (ArrayPub value : ArrayPub_values) {
		// add the value to a JSON object and serialize it.
		json j;
		j["value"] = value;
		std::string serialized;
		REQUIRE_NOTHROW(serialized = j.dump());

		// deserialize the JSON message, get the value, and check it.
		json deserialized;
		REQUIRE_NOTHROW(deserialized = json::parse(serialized));
		PointPub check = deserialized.at("value").get<PointPub>();
		REQUIRE(check == PP_INVALID);
	}	
}

TEST_CASE("PointPub cannot be deserialized to ArrayPub") {
	// go through all possible PointPub values
	for (PointPub value : PointPub_values) {
		// add the value to a JSON object and serialize it.
		json j;
		j["value"] = value;
		std::string serialized;
		REQUIRE_NOTHROW(serialized = j.dump());

		// deserialize the JSON message, get the value, and check it.
		json deserialized;
		REQUIRE_NOTHROW(deserialized = json::parse(serialized));
		ArrayPub check = deserialized.at("value").get<ArrayPub>();
		REQUIRE(check == AP_INVALID);
	}	
}

} // namespace rospub
