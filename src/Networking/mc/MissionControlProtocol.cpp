#include "MissionControlProtocol.h"

#include "../../Constants.h"

#include <unordered_set>

namespace proto {

using val_t = json::value_t;

/**
   Check if the given json object has the given key.
 */
static constexpr bool hasKey(const json& j, const std::string& key);
/**
   Check if the given json object has the given key with the given type.
 */
static constexpr bool validateKey(const json& j, const std::string& key, const val_t& type);
/**
   Check if the value in the given json object at the given key is a string in the given set of
   allowed values.
 */
static constexpr bool validateOneOf(const json& j, const std::string& key,
									const std::unordered_set<std::string>& vals);
/**
   Check if the value in the given json object at the given key is a floating-point number
   between min and max, inclusive.
 */
static constexpr bool validateRange(const json& j, const std::string& key, double min,
									double max);

bool validateEmergencyStopRequest(const json& j) {
	return validateKey(j, "stop", val_t::boolean);
}

bool validateOperationModeRequest(const json& j) {
	return validateKey(j, "mode", val_t::string) &&
		   validateOneOf(j, "mode", {"teleoperation", "autonomous"});
}

bool validateDriveRequest(const json& j) {
	return hasKey(j, "straight") && validateRange(j, "straight", -1, 1) &&
		   hasKey(j, "steer") && validateRange(j, "steer", -1, 1);
}

bool validateMotorPowerRequest(const json& j) {
	return validateKey(j, "motor", val_t::string) && validateRange(j, "power", -1, 1);
}

bool validateMotorPositionRequest(const json& j) {
	return validateKey(j, "motor", val_t::string) &&
		   validateKey(j, "position", val_t::number_float);
}

bool validateCameraStreamOpenRequest(const json& j) {
	return validateKey(j, "camera", val_t::string) &&
		   validateKey(j, "fps", val_t::number_unsigned) &&
		   validateKey(j, "width", val_t::number_unsigned) &&
		   validateKey(j, "height", val_t::number_unsigned);
}

bool validateCameraStreamCloseRequest(const json& j) {
	return validateKey(j, "camera", val_t::string);
}

websocket::WebSocketProtocol initMissionControlProtocol() {
	websocket::WebSocketProtocol proto(Constants::MC_PROTOCOL_NAME);
}

///// UTILITY FUNCTIONS //////

static constexpr bool hasKey(const json& j, const std::string& key) {
	return j.contains(key);
}

static constexpr bool validateKey(const json& j, const std::string& key, const val_t& type) {
	return hasKey(j, key) && j.at(key).type() == type;
}

static constexpr bool validateOneOf(const json& j, const std::string& key,
									const std::unordered_set<std::string>& vals) {
	return validateKey(j, key, val_t::string) &&
		   vals.find(static_cast<std::string>(j)) != vals.end();
}

static constexpr bool validateRange(const json& j, const std::string& key, double min,
									double max) {
	return j.type() == val_t::number_float && min <= static_cast<double>(j) &&
		   static_cast<double>(j) <= max;
}

} // namespace proto
