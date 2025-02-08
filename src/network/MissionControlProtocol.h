#pragma once

#include "../autonomous/AutonomousTask.h"
#include "../utils/scheduler.h"
#include "../video/H264Encoder.h"
#include "../world_interface/world_interface.h"
#include "MissionControlTasks.h"
#include "websocket/WebSocketProtocol.h"
#include "websocket/WebSocketServer.h"

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
using robot::types::motorid_t;
using websocket::SingleClientWSServer;
using websocket::WebSocketProtocol;

class MissionControlProtocol : public WebSocketProtocol { // TODO: add documentation
public:
	MissionControlProtocol(SingleClientWSServer& server);
	~MissionControlProtocol();
	MissionControlProtocol(const MissionControlProtocol& other) = delete;
	MissionControlProtocol& operator=(const MissionControlProtocol& other) = delete;

private:
	SingleClientWSServer& _server;
	tasks::PowerRepeatTask _power_repeat_task;
	tasks::CameraStreamTask _camera_stream_task;
	tasks::TelemReportTask _telem_report_task;
	tasks::ArmIKTask _arm_ik_task;
	autonomous::AutonomousTask _autonomous_task;

	void handleEmergencyStopRequest(const json& j);
	void handleOperationModeRequest(const json& j);
	void handleDriveModeRequest(const json& j);
	void handleTankDriveRequest(const json& j);
	void handleTurnInPlaceDriveRequest(const json& j);
	void handleCrabDriveRequest(const json& j);
	void handleCameraStreamOpenRequest(const json& j);
	void handleCameraStreamCloseRequest(const json& j);
	void handleJointPowerRequest(const json& j);
	void handleMotorPowerRequest(const json& j);
	void handleWaypointNavRequest(const json& j);
	void handleDriveRequest(const json& j);
	void handleRequestArmIKEnabled(const json& j);
	void sendArmIKEnabledReport(bool enabled);
	void handleConnection();
	void handleHeartbeatTimedOut();
	void stopAndShutdownPowerRepeat(bool sendDisableIK);
	void setArmIKEnabled(bool enabled, bool sendReport = true);
	void setRequestedJointPower(jointid_t joint, double power);
	void setRequestedMotorPower(motorid_t motor, double power);
	void setRequestedCmdVel(double dtheta, double dx);
	void setRequestedTankCmdVel(double left, double right);
	void setRequestedTurnInPlaceCmdVel(double dtheta);
	void setRequestedCrabCmdVel(double dtheta, double dy);
};

}; // namespace mc
} // namespace net
