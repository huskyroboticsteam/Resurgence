#pragma once

#include "../world_interface/world_interface.h"
#include "websocket/WebSocketProtocol.h"
#include "websocket/WebSocketServer.h"
#include "../video/H264Encoder.h"

#include <atomic>
#include <condition_variable>
#include <memory>
#include <optional>
#include <shared_mutex>
#include <thread>
#include <unordered_map>
#include <unordered_set>

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
	void jointPosReportTask();
	SingleClientWSServer& _server;
	std::shared_mutex _stream_mutex;
	std::unordered_map<CameraID, uint32_t> _open_streams;
	std::unordered_map<CameraID, std::shared_ptr<video::H264Encoder>> _camera_encoders;
	std::atomic<bool> _streaming_running;
	std::thread _streaming_thread;
	// for joint position reporting.
	std::thread _joint_report_thread;
	// protects _last_joint_power, _last_cmd_vel
	std::mutex _joint_power_mutex;
	std::unordered_map<jointid_t, double> _last_joint_power;
	// if not present, then there is no last requested drive power
	std::optional<std::pair<double, double>> _last_cmd_vel;
	// protects _joint_repeat_running, ALWAYS lock before thread and joint_power_mutex
	std::mutex _joint_repeat_running_mutex;
	bool _joint_repeat_running;
	// protects _joint_repeat_thread
	std::mutex _joint_repeat_thread_mutex;
	std::thread _joint_repeat_thread;
	std::condition_variable _power_repeat_cv;
	void handleEmergencyStopRequest(const json& j);
	void handleOperationModeRequest(const json& j);
	void handleCameraStreamOpenRequest(const json& j);
	void handleCameraStreamCloseRequest(const json& j);
	void handleJointPowerRequest(const json& j);
	void handleDriveRequest(const json& j);
	void sendJointPositionReport(const std::string& jointName, int32_t position);
	void sendCameraStreamReport(const CameraID& cam, const std::basic_string<uint8_t>& nal_data);
	void handleConnection();
	void startPowerRepeat();
	void stopAndShutdownPowerRepeat();
	void setRequestedJointPower(jointid_t joint, double power);
	void setRequestedCmdVel(double dtheta, double dx);
};

}; // namespace mc
} // namespace net
