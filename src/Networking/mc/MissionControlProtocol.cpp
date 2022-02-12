#include "MissionControlProtocol.h"

#include "../../Constants.h"

#include <unordered_map>
#include <unordered_set>

namespace mc {

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

static std::unordered_set<CameraID> open_camera_streams;
std::optional<websocket::WebSocketProtocol> proto;

bool validateEmergencyStopRequest(const json& j) {
	return validateKey(j, "stop", val_t::boolean);
}

void handleEmergencyStopRequest(const json& j) {}

bool validateOperationModeRequest(const json& j) {
	return validateKey(j, "mode", val_t::string) &&
		   validateOneOf(j, "mode", {"teleoperation", "autonomous"});
}

void handleOperationModeRequest(const json& j) {}

bool validateDriveRequest(const json& j) {
	return hasKey(j, "straight") && validateRange(j, "straight", -1, 1) &&
		   hasKey(j, "steer") && validateRange(j, "steer", -1, 1);
}

void handleDriveRequest(const json& j) {}

bool validateMotorPowerRequest(const json& j) {
	return validateKey(j, "motor", val_t::string) && validateRange(j, "power", -1, 1);
}

void handleMotorPowerRequest(const json& j) {}

bool validateMotorPositionRequest(const json& j) {
	return validateKey(j, "motor", val_t::string) &&
		   validateKey(j, "position", val_t::number_float);
}

void handleMotorPositionRequest(const json& j) {}

bool validateCameraStreamOpenRequest(const json& j) {
	return validateKey(j, "camera", val_t::string) &&
		   validateKey(j, "fps", val_t::number_unsigned) &&
		   validateKey(j, "width", val_t::number_unsigned) &&
		   validateKey(j, "height", val_t::number_unsigned);
}

void handleCameraStreamOpenRequest(const json& j) {}

bool validateCameraStreamCloseRequest(const json& j) {
	return validateKey(j, "camera", val_t::string);
}

void handleCameraStreamCloseRequest(const json& j) {}

websocket::WebSocketProtocol initMissionControlProtocol() {
	if (!proto) {
		proto = websocket::WebSocketProtocol(Constants::MC_PROTOCOL_NAME);
		proto->addMessageHandler(EMERGENCY_STOP_REQ_TYPE, handleEmergencyStopRequest,
								 validateEmergencyStopRequest);
		proto->addMessageHandler(OPERATION_MODE_REQ_TYPE, handleOperationModeRequest,
								 validateOperationModeRequest);
		proto->addMessageHandler(DRIVE_REQ_TYPE, handleDriveRequest, validateDriveRequest);
		proto->addMessageHandler(MOTOR_POWER_REQ_TYPE, handleMotorPowerRequest,
								 validateMotorPowerRequest);
		proto->addMessageHandler(CAMERA_STREAM_OPEN_REQ_TYPE, handleCameraStreamOpenRequest,
								 validateCameraStreamOpenRequest);
		proto->addMessageHandler(CAMERA_STREAM_CLOSE_REQ_TYPE, handleCameraStreamCloseRequest,
								 validateCameraStreamCloseRequest);
	}

	return *proto;
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
