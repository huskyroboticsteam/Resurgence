#include "ArduPilotProtocol.h"

#include "../Constants.h"
#include "../Globals.h"
#include "../log.h"
#include "../utils/json.h"

#include <mutex>

namespace net {
namespace ardupilot {

using json = nlohmann::json;
using val_t = json::value_t;
using namespace robot::types;
using namespace navtypes;
using std::placeholders::_1;
using namespace net::websocket;

ArduPilotProtocol::ArduPilotProtocol() {
	initArduPilotServer(Globals::websocketServer);
}

void ArduPilotProtocol::initArduPilotServer(SingleClientWSServer& websocketServer) {
	auto ardupilot_protocol = std::make_unique<net::websocket::WebSocketProtocol>(
		Constants::ARDUPILOT_PROTOCOL_NAME);
	ardupilot_protocol->addMessageHandler(
		"arduPilotGPSReport", std::bind(&ArduPilotProtocol::handleGPSRequest, this, _1),
		validateGPSRequest);
	ardupilot_protocol->addMessageHandler(
		"arduPilotIMUReport", std::bind(&ArduPilotProtocol::handleIMURequest, this, _1),
		validateIMURequest);
	ardupilot_protocol->addMessageHandler(
		"arduPilotHeadingReport",
		std::bind(&ArduPilotProtocol::handleHeadingRequest, this, _1), validateHeadingRequest);
	ardupilot_protocol->addConnectionHandler(
		std::bind(&ArduPilotProtocol::clientConnected, this));
	ardupilot_protocol->addDisconnectionHandler(
		std::bind(&ArduPilotProtocol::clientDisconnected, this));

	websocketServer.addProtocol(std::move(ardupilot_protocol));
}

void ArduPilotProtocol::clientConnected() {
	log(LOG_INFO, "ArduPilot connected.\n");
	{
		std::lock_guard<std::mutex> lock(_connectionMutex);
		_arduPilotProtocolConnected = true;
	}
}

void ArduPilotProtocol::clientDisconnected() {
	log(LOG_WARN, "ArduPilot disconnected.\n");
	{
		std::lock_guard<std::mutex> lock(_connectionMutex);
		_arduPilotProtocolConnected = false;
	}
}

bool ArduPilotProtocol::validateGPSRequest(const json& j) {
	return util::validateKey(j, "lat", val_t::number_float) &&
		   util::validateKey(j, "lon", val_t::number_float);
}

void ArduPilotProtocol::handleGPSRequest(const json& j) {
	double lat = j["lat"];
	double lon = j["lat"];
	{
		std::lock_guard<std::mutex> lock(_lastGPSMutex);
		_lastGPS = gpscoords_t{lat, lon};
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
	_lastOrientation = eulerangles_t{roll, pitch, yaw};
}

bool ArduPilotProtocol::validateHeadingRequest(const json& j) {
	return util::validateKey(j, "heading", val_t::number_integer);
}

void ArduPilotProtocol::handleHeadingRequest(const json& j) {
	int heading = j["heading"];
	_lastHeading = heading;
}

DataPoint<gpscoords_t> ArduPilotProtocol::getGPS() {
	std::unique_lock<std::mutex> lock(_lastGPSMutex);
	return _lastGPS;
}

DataPoint<eulerangles_t> ArduPilotProtocol::getIMU() {
	std::unique_lock<std::mutex> lock(_lastOrientationMutex);
	return _lastOrientation;
}

DataPoint<int> ArduPilotProtocol::getHeading() {
	std::unique_lock<std::mutex> lock(_lastHeadingMutex);
	return _lastHeading;
}

bool ArduPilotProtocol::isArduPilotConnected() {
	return _arduPilotProtocolConnected;
}

} // namespace ardupilot
} // namespace net
