#include "MissionControlTasks.h"

#include "../Constants.h"
#include "../Globals.h"
#include "../control_interface.h"
#include "../utils/core.h"
#include "../world_interface/world_interface.h"
#include "MissionControlMessages.h"

#include <loguru.hpp>

#include <nlohmann/json.hpp>

using namespace robot::types;
using namespace std::chrono_literals;
using Globals::swerveController;

using control::DriveMode;
using nlohmann::json;

namespace net::mc::tasks {
namespace {
const std::chrono::milliseconds TELEM_REPORT_PERIOD = 100ms;
}

/**
 * Get the steer rotations of all wheels as a swerve_rots_t.
 */
static DataPoint<control::swerve_rots_t> getAllSteerRotations();

/**
 * Set the drive power of all wheels with a drive_commands_t.
 */
static void setAllDrivePowers(const control::drive_commands_t& commands);

PowerRepeatTask::PowerRepeatTask()
	: util::PeriodicTask<>(Constants::JOINT_POWER_REPEAT_PERIOD,
						   std::bind(&PowerRepeatTask::periodicTask, this)) {}

void PowerRepeatTask::setJointPower(jointid_t id, double power) {
	std::lock_guard lock(_mutex);
	_last_joint_power[id] = power;
}

void PowerRepeatTask::setMotorPower(motorid_t id, double power) {
	std::lock_guard lock(_mutex);
	_last_motor_power[id] = power;
}

void PowerRepeatTask::setCmdVel(double steerVel, double xVel) {
	std::lock_guard lock(_mutex);
	_last_cmd_vel = {steerVel, xVel};
	_tank = false;
}

void PowerRepeatTask::setTankCmdVel(double left, double right) {
	std::lock_guard lock(_mutex);
	_last_cmd_vel = {left, right};
	_tank = true;
}

void PowerRepeatTask::setTurnInPlaceCmdVel(double steerVel) {
	std::lock_guard lock(_mutex);
	_last_cmd_vel = {steerVel, 0.0};
	DataPoint<control::swerve_rots_t> curr_rot = getAllSteerRotations();
	if (curr_rot.isValid()) {
		setAllDrivePowers(
			Globals::swerveController.setTurnInPlaceCmdVel(steerVel, curr_rot.getData()));
	}
}

void PowerRepeatTask::setCrabCmdVel(double dtheta, double yVel) {
	std::lock_guard lock(_mutex);
	_last_cmd_vel = {dtheta, yVel};
	DataPoint<control::swerve_rots_t> curr_rot = getAllSteerRotations();
	if (curr_rot.isValid()) {
		setAllDrivePowers(
			Globals::swerveController.setCrabCmdVel(dtheta, yVel, curr_rot.getData()));
	}
}

void PowerRepeatTask::start() {
	// lock the mutex before start() to prevent data races between start() and stop()
	std::lock_guard lock(_mutex);
	util::PeriodicTask<>::start();
}

void PowerRepeatTask::stop() {
	std::lock_guard lock(_mutex);
	util::PeriodicTask<>::stop();
	_last_joint_power.clear();
	_last_cmd_vel.reset();
}

void PowerRepeatTask::periodicTask() {
	std::lock_guard lock(_mutex);
	for (const auto& current_pair : _last_joint_power) {
		const jointid_t& joint = current_pair.first;
		const double& power = current_pair.second;
		// no need to repeatedly send 0 power
		// this is also needed to make the zero calibration script work
		if (power != 0.0) {
			robot::setJointPower(joint, power);
		}
	}
	for (const auto& current_pair : _last_motor_power) {
		const motorid_t& motor = current_pair.first;
		const double& power = current_pair.second;
		if (power != 0.0) {
			robot::setMotorPower(motor, power);
		}
	}
	if (_last_cmd_vel) {
		if (_last_cmd_vel->first != 0.0 || _last_cmd_vel->second != 0.0) {
			if (swerveController.getDriveMode() == DriveMode::Normal) {
				if (_tank) {
					robot::setTankCmdVel(_last_cmd_vel->first, _last_cmd_vel->second);
				} else {
					robot::setCmdVel(_last_cmd_vel->first, _last_cmd_vel->second);
				}
			} else if (swerveController.getDriveMode() == DriveMode::TurnInPlace) {
				DataPoint<control::swerve_rots_t> curr_rot = getAllSteerRotations();
				if (curr_rot.isValid()) {
					setAllDrivePowers(Globals::swerveController.setTurnInPlaceCmdVel(
						_last_cmd_vel->first, curr_rot.getData()));
				}
			} else {
				DataPoint<control::swerve_rots_t> curr_rot = getAllSteerRotations();
				if (curr_rot.isValid()) {
					setAllDrivePowers(Globals::swerveController.setCrabCmdVel(
						_last_cmd_vel->first, _last_cmd_vel->second, curr_rot.getData()));
				}
			}
		}
	}
}

CameraStreamTask::CameraStreamTask(websocket::SingleClientWSServer& server)
	: util::AsyncTask<>("MCP_Stream"), _server(server) {}

void CameraStreamTask::openStream(const CameraID& cam, int fps) {
	std::lock_guard lock(_mutex);
	_open_streams[cam] = 0;
	auto it = Constants::video::STREAM_RFS.find(cam);
	int rf = it != Constants::video::STREAM_RFS.end() ? it->second
													  : Constants::video::H264_RF_CONSTANT;
	_camera_encoders[cam] = std::make_shared<video::H264Encoder>(fps, rf);
}

void CameraStreamTask::closeStream(const CameraID& cam) {
	std::lock_guard lock(_mutex);
	_open_streams.erase(cam);
	_camera_encoders.erase(cam);
}

void CameraStreamTask::task(std::unique_lock<std::mutex>&) {
	while (isRunningInternal()) {
		{
			std::lock_guard lg(_mutex);
			// for all open streams, check if there is a new frame
			for (const auto& stream : _open_streams) {
				const CameraID& cam = stream.first;
				const uint32_t& frame_num = stream.second;
				if (robot::hasNewCameraFrame(cam, frame_num)) {
					// if there is a new frame, grab it
					auto camDP = robot::readCamera(cam);

					if (camDP) {
						auto data = camDP.getData();
						uint32_t& new_frame_num = data.second;
						cv::Mat frame = data.first;
						// update the previous frame number
						this->_open_streams[cam] = new_frame_num;
						const auto& encoder = this->_camera_encoders[cam];

						// convert frame to encoded data and send it
						auto data_vector = encoder->encode_frame(frame);
						json msg = {{"type", CAMERA_STREAM_REP_TYPE},
									{"camera", cam},
									{"data", data_vector}};
						_server.sendJSON(Constants::MC_PROTOCOL_NAME, msg);
					}
				}
			}
		}
		std::this_thread::yield();
	}
}

TelemReportTask::TelemReportTask(websocket::SingleClientWSServer& server)
	: util::PeriodicTask<>(TELEM_REPORT_PERIOD,
						   std::bind(&TelemReportTask::sendTelemetry, this)),
	  _server(server) {}

void TelemReportTask::sendTelemetry() {
	// send joint positions
	for (const auto& cur : robot::types::name_to_jointid) {
		robot::types::DataPoint<int32_t> jpos = robot::getJointPos(cur.second);
		if (jpos.isValid()) {
			auto jointNameFrznStr = cur.first;
			std::string jointNameStdStr(jointNameFrznStr.data(), jointNameFrznStr.size());
			json msg = {{"type", JOINT_POSITION_REP_TYPE},
						{"joint", jointNameStdStr},
						{"position", static_cast<double>(jpos.getData()) / 1000.0}};
			this->_server.sendJSON(Constants::MC_PROTOCOL_NAME, msg);
		}
	}

	// send rover position and orientation
	auto imu = robot::readIMU();
	auto gps = gps::readGPSCoords();
	LOG_F(2, "imu_valid=%s, gps_valid=%s", util::to_string(imu.isValid()).c_str(),
		  util::to_string(gps.isValid()).c_str());
	if (imu.isValid()) {
		Eigen::Quaterniond quat = imu.getData();
		double lon = 0, lat = 0;
		if (gps.isValid()) {
			lon = gps.getData().lon;
			lat = gps.getData().lat;
		}
		double imuRecency = util::durationToSec(dataclock::now() - imu.getTime());
		double recency = imuRecency;
		if (gps.isValid()) {
			double gpsRecency = util::durationToSec(dataclock::now() - gps.getTime());
			recency = std::max(recency, gpsRecency);
		}
		json msg = {{"type", ROVER_POS_REP_TYPE},
					{"orientW", quat.w()},
					{"orientX", quat.x()},
					{"orientY", quat.y()},
					{"orientZ", quat.z()},
					{"lon", lon},
					{"lat", lat},
					{"recency", recency}};
		_server.sendJSON(Constants::MC_PROTOCOL_NAME, msg);
	}
}

ArmIKTask::ArmIKTask(websocket::SingleClientWSServer& server)
	: util::PeriodicTask<>(Constants::ARM_IK_UPDATE_PERIOD,
						   std::bind(&ArmIKTask::updateArmIK, this)),
	  _server(server) {}

void ArmIKTask::updateArmIK() {
	DataPoint<navtypes::Vectord<Constants::arm::IK_MOTORS.size()>> armJointPositions =
		robot::getMotorPositionsRad(Constants::arm::IK_MOTORS);
	if (armJointPositions.isValid()) {
		navtypes::Vectord<Constants::arm::IK_MOTORS.size()> targetJointPositions =
			Globals::planarArmController.getCommand(dataclock::now(),
													armJointPositions.getData());
		targetJointPositions /= M_PI / 180.0 / 1000.0; // convert from radians to millidegrees
		for (size_t i = 0; i < Constants::arm::IK_MOTORS.size(); i++) {
			robot::setMotorPos(Constants::arm::IK_MOTORS[i],
							   static_cast<int32_t>(targetJointPositions(i)));
		}
	}
}

static DataPoint<control::swerve_rots_t> getAllSteerRotations() {
	try {
		return DataPoint<control::swerve_rots_t>(
			{robot::getMotorPos(motorid_t::frontLeftSwerve).getData(),
			 robot::getMotorPos(motorid_t::frontRightSwerve).getData(),
			 robot::getMotorPos(motorid_t::rearLeftSwerve).getData(),
			 robot::getMotorPos(motorid_t::rearRightSwerve).getData()});
	} catch (const bad_datapoint_access& e) {
		LOG_F(WARNING, "Invalid steer motor position(s)!");
		return {};
	}
}

static void setAllDrivePowers(const control::drive_commands_t& commands) {
	robot::setMotorPower(motorid_t::frontLeftWheel, commands.lfPower);
	robot::setMotorPower(motorid_t::frontRightWheel, commands.rfPower);
	robot::setMotorPower(motorid_t::rearLeftWheel, commands.lbPower);
	robot::setMotorPower(motorid_t::rearRightWheel, commands.rbPower);
}
} // namespace net::mc::tasks
