#pragma once

#include "../utils/scheduler.h"
#include "../video/H264Encoder.h"
#include "../world_interface/data.h"
#include "websocket/WebSocketServer.h"

#include <memory>
#include <mutex>
#include <optional>
#include <unordered_map>
#include <utility>

namespace net::mc::tasks {

/**
 * @brief This task is responsible for periodically resending setMotorPower commands to the
 * motors. This feeds the hardware watchdog and keeps the motors running.
 */
class PowerRepeatTask : public util::PeriodicTask<> {
public:
	PowerRepeatTask();

	/**
	 * @brief Set the power to resend to the given joint. This does not set the motor power
	 * right now.
	 */
	void setJointPower(robot::types::jointid_t id, double power);

	/**
	 * @brief Set the drive command to resend. This does not execute the drive command right
	 * now.
	 */
	void setCmdVel(double steerVel, double xVel);

	/**
	 * @brief Set the tank drive command to resend. This does not execute the drive command
	 * right now.
	 */
	void setTankCmdVel(double steerVel, double xVel);

	void start() override;

	void stop() override;

private:
	std::mutex _mutex;
	std::unordered_map<robot::types::jointid_t, double> _last_joint_power;
	// if not present, then there is no last requested drive power
	std::optional<std::pair<double, double>> _last_cmd_vel;
	bool _tank;

	void periodicTask();
};

/**
 * @brief This task is responsible for sending the camera streams to Mission Control over
 * websocket.
 */
class CameraStreamTask : public util::AsyncTask<> {
public:
	CameraStreamTask(websocket::SingleClientWSServer& server);

	/**
	 * @brief Start streaming a new camera to Mission Control.
	 *
	 * @param cam The camera to stream.
	 * @param fps The fps to stream at.
	 */
	void openStream(const robot::types::CameraID& cam, int fps);

	/**
	 * @brief Stop streaming a camera to Mission Control.
	 *
	 * @param cam The camera to stop streaming.
	 */
	void closeStream(const robot::types::CameraID& cam);

protected:
	void task(std::unique_lock<std::mutex>& lock) override;

private:
	websocket::SingleClientWSServer& _server;
	std::mutex _mutex;
	std::unordered_map<robot::types::CameraID, uint32_t> _open_streams;
	std::unordered_map<robot::types::CameraID, std::shared_ptr<video::H264Encoder>>
		_camera_encoders;
};

/**
 * @brief This task periodically sends robot telemetry data to Mission Control.
 */
class TelemReportTask : public util::PeriodicTask<> {
public:
	TelemReportTask(websocket::SingleClientWSServer& server);

private:
	websocket::SingleClientWSServer& _server;

	void sendTelemetry();
};

/**
 * @brief This task sets the arm joint positions to track the current IK command.
 *
 * This task should be enabled and disabled to be kept in sync with Globals::armIKEnabled
 */
class ArmIKTask : public util::PeriodicTask<> {
public:
	ArmIKTask(websocket::SingleClientWSServer& server);

private:
	std::mutex _mutex;
	websocket::SingleClientWSServer& _server;

	void updateArmIK();
};

} // namespace net::mc::tasks
