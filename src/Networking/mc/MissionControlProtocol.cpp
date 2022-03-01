#include "MissionControlProtocol.h"

#include "../../Constants.h"
#include "../../Globals.h"
#include "../../base64/base64_img.h"
#include "../../log.h"

#include <functional>
#include <unordered_map>
#include <unordered_set>
#include <opencv2/core.hpp>

namespace mc {

using val_t = json::value_t;
using std::placeholders::_1;
using websocket::connhandler_t;
using websocket::msghandler_t;
using websocket::validator_t;

// request keys
constexpr const char* EMERGENCY_STOP_REQ_TYPE = "emergencyStopRequest";
constexpr const char* OPERATION_MODE_REQ_TYPE = "operationModeRequest";
constexpr const char* DRIVE_REQ_TYPE = "driveRequest";
constexpr const char* MOTOR_POWER_REQ_TYPE = "motorPowerRequest";
constexpr const char* MOTOR_POSITION_REQ_TYPE = "motorPositionRequest";
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

/*///////////////// VALIDATORS/HANDLERS ////////////////////

  For each protocol message type, there is a pair of functions in this section: a validator and
  a handler. The validator will return a boolean depending on whether or not a message is valid
  for this type, and the handler will perform the required actions for dealing with a message
  of the type. NOTE: handlers expect valid messages, so call validator first
*/

static bool validateEmergencyStopRequest(const json& j) {
	return validateKey(j, "stop", val_t::boolean);
}

static void handleEmergencyStopRequest(const json& j) {
	bool stop = j["stop"];
	if (stop) {
		setCmdVel(0, 0);
	}
	Globals::E_STOP = stop;
}

static bool validateOperationModeRequest(const json& j) {
	return validateKey(j, "mode", val_t::string) &&
		   validateOneOf(j, "mode", {"teleoperation", "autonomous"});
}

static void handleOperationModeRequest(const json& j) {
	std::string mode = j["mode"];
	Globals::AUTONOMOUS = (mode == "autonomous");
}

static bool validateDriveRequest(const json& j) {
	std::string msg = j.dump();
	return hasKey(j, "straight") && validateRange(j, "straight", -1, 1) &&
		   hasKey(j, "steer") && validateRange(j, "steer", -1, 1);
}

static void handleDriveRequest(const json& j) {
	// fit straight and steer to unit circle; i.e. scale each such that <straight, steer> is a
	// unit vector
	double straight = j["straight"];
	double steer = j["steer"];
	double norm = std::sqrt(std::pow(straight, 2) + std::pow(steer, 2));
	double dx = Constants::MAX_WHEEL_VEL * (norm > 1e-6 ? straight / norm : straight);
	double dtheta = Constants::MAX_DTHETA * (norm > 1e-6 ? -steer / norm : steer);
	log(LOG_INFO, "{straight=%.2f, steer=%.2f} -> setCmdVel(%.4f, %.4f)\n", straight, steer,
		dtheta, dx);
	setCmdVel(dtheta, dx);
}

static bool validateMotorPowerRequest(const json& j) {
	return validateKey(j, "motor", val_t::string) && validateRange(j, "power", -1, 1);
}

static void handleMotorPowerRequest(const json& j) {
	std::string motor = j["motor"];
	double power = j["power"];
	setMotorPWM(motor, power);
}

static bool validateMotorPositionRequest(const json& j) {
	return validateKey(j, "motor", val_t::string) &&
		   validateKey(j, "position", val_t::number_integer);
}

static void handleMotorPositionRequest(const json& j) {
	std::string motor = j["motor"];
	double position_deg = j["position"];
	int32_t position_mdeg = std::round(position_deg * 1000);
	setMotorPos(motor, position_mdeg);
}

static bool validateCameraStreamOpenRequest(const json& j) {
	return validateKey(j, "camera", val_t::string);
}

void MissionControlProtocol::handleCameraStreamOpenRequest(const json& j) {
	CameraID cam = j["camera"];
	this->_open_streams[cam] = 0;
}

static bool validateCameraStreamCloseRequest(const json& j) {
	return validateKey(j, "camera", val_t::string);
}

void MissionControlProtocol::handleCameraStreamCloseRequest(const json& j) {
	CameraID cam = j["camera"];
	this->_open_streams.erase(cam);
}

void MissionControlProtocol::sendCameraStreamReport(const CameraID& cam,
													const std::string& b64_data) {
	json msg = {{"type", CAMERA_STREAM_REP_TYPE}, {"camera", cam}, {"data", b64_data}};
	this->_server.sendJSON(Constants::MC_PROTOCOL_NAME, msg);
}

void MissionControlProtocol::videoStreamTask() {
	_streaming_running = true;
	while (_streaming_running) {
		// for all open streams, check if there is a new frame
		for (const auto& stream : _open_streams) {
			const CameraID& cam = stream.first;
			const uint32_t& frame_num = stream.second;
			if (hasNewCameraFrame(cam, frame_num)) {
				// if there is a new frame, grab it
				auto data = readCamera(cam).getData();
				uint32_t& new_frame_num = data.second;
				cv::Mat frame = data.first;
				// update the previous frame number
				this->_open_streams[cam] = new_frame_num;

				// convert frame to base64 and send it
				std::string b64_data = base64::encodeMat(frame, "jpg");
				sendCameraStreamReport(cam, b64_data);
			}

			// break out of the loop if we should stop streaming
			if (!_streaming_running) {
				break;
			}
		}
	}
}

MissionControlProtocol::MissionControlProtocol(SingleClientWSServer& server)
	: WebSocketProtocol(Constants::MC_PROTOCOL_NAME), _server(server) {
	this->addMessageHandler(EMERGENCY_STOP_REQ_TYPE, handleEmergencyStopRequest,
							validateEmergencyStopRequest);
	this->addMessageHandler(OPERATION_MODE_REQ_TYPE, handleOperationModeRequest,
							validateOperationModeRequest);
	this->addMessageHandler(DRIVE_REQ_TYPE, handleDriveRequest, validateDriveRequest);
	this->addMessageHandler(MOTOR_POWER_REQ_TYPE, handleMotorPowerRequest,
							validateMotorPowerRequest);
	this->addMessageHandler(MOTOR_POSITION_REQ_TYPE, handleMotorPositionRequest,
							validateMotorPositionRequest);
	// camera stream handlers need the class for context since they must modify _open_streams
	this->addMessageHandler(
		CAMERA_STREAM_OPEN_REQ_TYPE,
		std::bind(&MissionControlProtocol::handleCameraStreamOpenRequest, this, _1),
		validateCameraStreamOpenRequest);
	this->addMessageHandler(
		CAMERA_STREAM_CLOSE_REQ_TYPE,
		std::bind(&MissionControlProtocol::handleCameraStreamCloseRequest, this, _1),
		validateCameraStreamCloseRequest);

	this->_streaming_running = true;
	this->_streaming_thread = std::thread(&MissionControlProtocol::videoStreamTask, this);
}

MissionControlProtocol::~MissionControlProtocol() {
	this->_streaming_running = false;
	if(this->_streaming_thread.joinable()){
		this->_streaming_thread.join();
	}
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
