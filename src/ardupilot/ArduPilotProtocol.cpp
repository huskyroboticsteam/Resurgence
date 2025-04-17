#include "ArduPilotProtocol.h"

#include "../Constants.h"
#include "../utils/json.h"
#include "../utils/transform.h"

#include <loguru.hpp>
#include <mutex>

namespace net {
namespace ardupilot {

using json = nlohmann::json;
using val_t = json::value_t;
using namespace robot::types;
using namespace navtypes;
using std::placeholders::_1;
using namespace net::websocket;

ArduPilotProtocol::ArduPilotProtocol(net::websocket::SingleClientWSServer& websocketServer) {
	initArduPilotServer(websocketServer);
}

ArduPilotProtocol::~ArduPilotProtocol() {}

void ArduPilotProtocol::initArduPilotServer(SingleClientWSServer& websocketServer) {
	auto ardupilot_protocol = std::make_unique<net::websocket::WebSocketProtocol>(
		Constants::ARDUPILOT_PROTOCOL_NAME);
	ardupilot_protocol->addMessageHandler(
		"gps", std::bind(&ArduPilotProtocol::handleGPSRequest, this, _1), validateGPSRequest);
	ardupilot_protocol->addMessageHandler(
		"orientation", std::bind(&ArduPilotProtocol::handleIMURequest, this, _1),
		validateIMURequest);
	ardupilot_protocol->addConnectionHandler(
		std::bind(&ArduPilotProtocol::clientConnected, this));
	ardupilot_protocol->addDisconnectionHandler(
		std::bind(&ArduPilotProtocol::clientDisconnected, this));

	websocketServer.addProtocol(std::move(ardupilot_protocol));
}

void ArduPilotProtocol::clientConnected() {
	LOG_F(INFO, "ArduPilot connected.");
	{
		std::lock_guard<std::mutex> lock(_connectionMutex);
		_arduPilotProtocolConnected = true;
	}
}

void ArduPilotProtocol::clientDisconnected() {
	LOG_F(WARNING, "ArduPilot disconnected.");
	{
		std::lock_guard<std::mutex> lock(_connectionMutex);
		_arduPilotProtocolConnected = false;
	}
}

bool ArduPilotProtocol::validateGPSRequest(const json& j) {
	return util::validateKey(j, "lat", val_t::number_float) &&
		   util::validateKey(j, "lon", val_t::number_float) &&
		   util::validateKey(j, "alt", val_t::number_float);
}

void ArduPilotProtocol::handleGPSRequest(const json& j) {
	double lat = j["lat"];
	double lon = j["lon"];
	double alt = j["alt"];
	{
		std::lock_guard<std::mutex> lock(_lastGPSMutex);
		_lastGPS = gpscoords_t{lat, lon, alt};
	}
}

bool ArduPilotProtocol::validateIMURequest(const json& j) {
	return util::validateKey(j, "roll", val_t::number_float) &&
		   util::validateKey(j, "pitch", val_t::number_float) &&
		   util::validateKey(j, "yaw", val_t::number_float);
}

void ArduPilotProtocol::handleIMURequest(const json& j) {
	double roll = j["roll"];
	double pitch = j["pitch"];
	double yaw = j["yaw"];
	yaw = -yaw;
	eulerangles_t rpy{roll, pitch, yaw};

	std::lock_guard lock(_lastOrientationMutex);
	_lastOrientation = util::eulerAnglesToQuat(rpy);
}

DataPoint<gpscoords_t> ArduPilotProtocol::getGPS() {
	std::unique_lock<std::mutex> lock(_lastGPSMutex);
	return _lastGPS;
}

DataPoint<Eigen::Quaterniond> ArduPilotProtocol::getIMU() {
	std::unique_lock<std::mutex> lock(_lastOrientationMutex);
	return _lastOrientation;
}

bool ArduPilotProtocol::isArduPilotConnected() {
	std::unique_lock<std::mutex> lock(_connectionMutex);
	return _arduPilotProtocolConnected;
}

} // namespace ardupilot
} // namespace net
