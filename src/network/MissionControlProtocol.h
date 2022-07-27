#pragma once

#include "websocket/WebSocketProtocol.h"
#include "websocket/WebSocketServer.h"
#include "../world_interface/world_interface.h"

#include <atomic>
#include <memory>
#include <shared_mutex>
#include <thread>
#include <unordered_map>
#include <unordered_set>

namespace net {
namespace mc {

using json = nlohmann::json;
using robot::types::CameraID;
using websocket::SingleClientWSServer;
using websocket::WebSocketProtocol;

class MissionControlProtocol : public WebSocketProtocol { // TODO: add documentation
public:
	MissionControlProtocol(SingleClientWSServer& server);
	~MissionControlProtocol();
	MissionControlProtocol(const MissionControlProtocol& other) = delete;
	MissionControlProtocol& operator=(const MissionControlProtocol& other) = delete;
	std::unordered_set<CameraID> getOpenCameraStreams();

private:
	void videoStreamTask();
	SingleClientWSServer& _server;
	std::shared_mutex _stream_mutex;
	std::unordered_map<CameraID, uint32_t> _open_streams;
	std::atomic<bool> _streaming_running;
	std::thread _streaming_thread;
	void handleCameraStreamOpenRequest(const json& j);
	void handleCameraStreamCloseRequest(const json& j);
	void sendCameraStreamReport(const CameraID& cam, const std::string& b64_data);
	void handleConnection();
};

}; // namespace mc
} // namespace net
