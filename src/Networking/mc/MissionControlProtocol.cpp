#include "MissionControlProtocol.h"

#include "../../Constants.h"
#include "../../Globals.h"
#include "../../log.h"

#include <unordered_map>
#include <unordered_set>

namespace mc {

using val_t = json::value_t;
// request keys
constexpr const char* EMERGENCY_STOP_REQ_TYPE = "emergencyStopRequest";
constexpr const char* OPERATION_MODE_REQ_TYPE = "operationModeRequest";
constexpr const char* DRIVE_REQ_TYPE = "driveRequest";
constexpr const char* MOTOR_POWER_REQ_TYPE = "motorPowerRequest";
constexpr const char* CAMERA_STREAM_OPEN_REQ_TYPE = "cameraStreamOpenRequest";
constexpr const char* CAMERA_STREAM_CLOSE_REQ_TYPE = "cameraStreamCloseRequest";

// report keys
constexpr const char* MOTOR_STATUS_REP_TYPE = "motorStatusReport";
constexpr const char* CAMERA_STREAM_REP_TYPE = "cameraStreamReport";
constexpr const char* LIDAR_REP_TYPE = "lidarReport";

/**
   Check if the given json object has the given key.
 */
static bool hasKey(const json& j, const std::string& key);
/**
   Check if the given json object has the given key with the given type.
 */
static bool validateKey(const json& j, const std::string& key, const val_t& type);
/**
   Check if the given json object has the given key, with a type in the given set of types.
 */
static bool validateKey(const json& j, const std::string& key,
						const std::unordered_set<val_t>& types);
/**
   Check if the value in the given json object at the given key is a string in the given set of
   allowed values.
 */
static bool validateOneOf(const json& j, const std::string& key,
						  const std::unordered_set<std::string>& vals);
/**
   Check if the value in the given json object at the given key is a floating-point number
   between min and max, inclusive.
 */
static bool validateRange(const json& j, const std::string& key, double min, double max);

static std::unordered_set<CameraID> open_camera_streams;
std::optional<websocket::WebSocketProtocol> proto;

/*///////////////// VALIDATORS/HANDLERS ////////////////////

  For each protocol message type, there is a pair of functions in this section: a validator and
  a handler. The validator will return a boolean depending on whether or not a message is valid
  for this type, and the handler will perform the required actions for dealing with a message
  of the type. NOTE: handlers expect valid messages, so call validator first
*/

bool validateEmergencyStopRequest(const json& j) {
	return validateKey(j, "stop", val_t::boolean);
}

void handleEmergencyStopRequest(const json& j) {
	bool stop = j["stop"];
	if (stop) {
		setCmdVel(0, 0);
	}
	Globals::E_STOP = stop;
}

bool validateOperationModeRequest(const json& j) {
	return validateKey(j, "mode", val_t::string) &&
		   validateOneOf(j, "mode", {"teleoperation", "autonomous"});
}

void handleOperationModeRequest(const json& j) {
	std::string mode = j["mode"];
	Globals::AUTONOMOUS = (mode == "autonomous");
}

bool validateDriveRequest(const json& j) {
	std::string msg = j.dump();
	return hasKey(j, "straight") && validateRange(j, "straight", -1, 1) &&
		   hasKey(j, "steer") && validateRange(j, "steer", -1, 1);
}

void handleDriveRequest(const json& j) {
	// fit straight and steer to unit circle; i.e. scale each such that <straight, steer> is a
	// unit vector
	double straight = j["straight"];
	double steer = j["steer"];
	double norm = std::sqrt(std::pow(straight, 2) + std::pow(steer, 2));
	double dx = Constants::MAX_WHEEL_VEL * (norm > 1e-6 ? straight / norm : straight);
	double dtheta = Constants::MAX_DTHETA * (norm > 1e-6 ? -steer / norm : steer);
	log(LOG_INFO, "{straight=%.2f, steer=%.2f} -> setCmdVel(%.4f, %.4f)\n", straight, steer, dtheta, dx);
	setCmdVel(dtheta, dx);
}

bool validateMotorPowerRequest(const json& j) {
	return validateKey(j, "motor", val_t::string) && validateRange(j, "power", -1, 1);
}

void handleMotorPowerRequest(const json& j) {
	std::string motor = j["motor"];
	double power = j["power"];
	setMotorPWM(motor, power);
}

bool validateMotorPositionRequest(const json& j) {
	return validateKey(j, "motor", val_t::string) &&
		   validateKey(j, "position", val_t::number_integer);
}

void handleMotorPositionRequest(const json& j) {
	std::string motor = j["motor"];
	double position_deg = j["position"];
	int32_t position_mdeg = std::round(position_deg * 1000);
	setMotorPos(motor, position_mdeg);
}

bool validateCameraStreamOpenRequest(const json& j) {
	return validateKey(j, "camera", val_t::string);
}

void handleCameraStreamOpenRequest(const json& j) {
	CameraID cam = j["camera"];
	open_camera_streams.insert(cam);
}

bool validateCameraStreamCloseRequest(const json& j) {
	return validateKey(j, "camera", val_t::string);
}

void handleCameraStreamCloseRequest(const json& j) {
	CameraID cam = j["camera"];
	open_camera_streams.erase(cam);
}

///////////////////////////////////////////////////////////////////

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

static bool hasKey(const json& j, const std::string& key) {
	return j.contains(key);
}

static bool validateKey(const json& j, const std::string& key, const val_t& type) {
	return hasKey(j, key) && j.at(key).type() == type;
}

static bool validateKey(const json& j, const std::string& key,
						const std::unordered_set<val_t>& types) {
	return hasKey(j, key) && types.find(j.at(key).type()) != types.end();
}

static bool validateOneOf(const json& j, const std::string& key,
						  const std::unordered_set<std::string>& vals) {
	return validateKey(j, key, val_t::string) &&
		   vals.find(static_cast<std::string>(j[key])) != vals.end();
}

static bool validateRange(const json& j, const std::string& key, double min, double max) {
	if (validateKey(j, key,
					{val_t::number_float, val_t::number_unsigned, val_t::number_integer})) {
		double d = j[key];
		return min <= d && d <= max;
	}
	return false;
}

} // namespace mc
