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
	void handleCameraStreamOpenRequest(const json& j);
	void handleCameraStreamCloseRequest(const json& j);
};

}; // namespace mc
