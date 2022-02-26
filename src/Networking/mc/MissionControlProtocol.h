#pragma once

#include "../../world_interface/world_interface.h"
#include "../websocket/WebSocketProtocol.h"
#include "../websocket/WebSocketServer.h"

#include <atomic>
#include <memory>
#include <unordered_set>

namespace mc {
using json = nlohmann::json;
using websocket::SingleClientWSServer;
using websocket::WebSocketProtocol;

class MissionControlProtocol : WebSocketProtocol {
public:
	MissionControlProtocol(SingleClientWSServer& server);
	~MissionControlProtocol();
	std::unordered_set<CameraID> getOpenCameraStreams();

private:
	void videoStreamTask();
	std::atomic<bool> _streaming_running;
	SingleClientWSServer& _server;
	std::unordered_set<CameraID> _open_streams;
	////// VALIDATORS //////
	bool validateEmergencyStopRequest(const json& j);
	bool validateOperationModeRequest(const json& j);
	bool validateDriveRequest(const json& j);
	bool validateMotorPowerRequest(const json& j);
	bool validateMotorPositionRequest(const json& j);
	bool validateCameraStreamOpenRequest(const json& j);
	bool validateCameraStreamCloseRequest(const json& j);
	////// HANDLERS //////
	void handleEmergencyStopRequest(const json& j);
	void handleOperationModeRequest(const json& j);
	void handleDriveRequest(const json& j);
	void handleMotorPowerRequest(const json& j);
	void handleMotorPositionRequest(const json& j);
	void handleCameraStreamOpenRequest(const json& j);
	void handleCameraStreamCloseRequest(const json& j);
};

}; // namespace mc
