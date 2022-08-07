#pragma once

#include "websocket/WebSocketProtocol.h"
#include "websocket/WebSocketServer.h"
#include "../world_interface/world_interface.h"

#include <atomic>
#include <condition_variable>
#include <memory>
#include <shared_mutex>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <optional>

namespace net {
namespace mc {

using json = nlohmann::json;
using robot::types::CameraID;
using robot::types::jointid_t;
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
	void jointPowerRepeatTask();
	SingleClientWSServer& _server;
	std::shared_mutex _stream_mutex;
	std::unordered_map<CameraID, uint32_t> _open_streams;
	std::atomic<bool> _streaming_running;
	std::thread _streaming_thread;
	// protects _last_joint_power, _last_cmd_vel
	std::mutex _joint_power_mutex;
	std::unordered_map<jointid_t, double> _last_joint_power;
	// if not present, then there is no last requested drive power
	std::optional<std::pair<double, double>> _last_cmd_vel;
	// protects _joint_repeat_running and _joint_repeat_thread
	std::mutex _joint_repeat_mutex;
	bool _joint_repeat_running;
	std::thread _joint_repeat_thread;
	std::condition_variable _power_repeat_cv;
	void handleEmergencyStopRequest(const json& j);
	void handleOperationModeRequest(const json& j);
	void handleCameraStreamOpenRequest(const json& j);
	void handleCameraStreamCloseRequest(const json& j);
	void handleJointPowerRequest(const json& j);
	void handleDriveRequest(const json& j);
	void sendCameraStreamReport(const CameraID& cam, const std::string& b64_data);
	void handleConnection();
	void startPowerRepeat();
	void stopAndShutdownPowerRepeat();
	void setRequestedJointPower(jointid_t joint, double power);
	void setRequestedCmdVel(double dtheta, double dx);
};

}; // namespace mc
} // namespace net
