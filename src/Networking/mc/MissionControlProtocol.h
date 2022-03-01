#pragma once

#include "../../world_interface/world_interface.h"
#include "../websocket/WebSocketProtocol.h"
#include "../websocket/WebSocketServer.h"

#include <atomic>
#include <memory>
#include <unordered_set>
#include <unordered_map>
#include <thread>
#include <mutex>

namespace mc {

using json = nlohmann::json;
using websocket::SingleClientWSServer;
using websocket::WebSocketProtocol;

class MissionControlProtocol : public WebSocketProtocol {
public:
	MissionControlProtocol(SingleClientWSServer& server);
	~MissionControlProtocol();
	MissionControlProtocol(const MissionControlProtocol& other) = delete;
	MissionControlProtocol& operator=(const MissionControlProtocol& other) = delete;
	std::unordered_set<CameraID> getOpenCameraStreams();

private:
	void videoStreamTask();
	SingleClientWSServer& _server;
	std::mutex _stream_lock;
	std::unordered_map<CameraID, uint32_t> _open_streams;
	std::atomic<bool> _streaming_running;
	std::thread _streaming_thread;
	void handleCameraStreamOpenRequest(const json& j);
	void handleCameraStreamCloseRequest(const json& j);
	void sendCameraStreamReport(const CameraID& cam, const std::string& b64_data);
	void handleConnection();
};

}; // namespace mc
