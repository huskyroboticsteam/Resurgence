#include "MissionControlTasks.h"

#include "../Constants.h"
#include "../Globals.h"
#include "../camera/CameraConfig.h"
#include "../control_interface.h"
#include "../utils/core.h"
#include "../world_interface/world_interface.h"
#include "MissionControlMessages.h"

#include <algorithm>
#include <cctype>
#include <loguru.hpp>
#include <vector>
#include <nlohmann/json.hpp>

using namespace robot::types;
using namespace std::chrono_literals;

using nlohmann::json;

namespace net::mc::tasks {
namespace {
const std::chrono::milliseconds TELEM_REPORT_PERIOD = 100ms;

template <class... Ts> struct overloaded : Ts... {
	using Ts::operator()...;
};
template <class... Ts> overloaded(Ts...) -> overloaded<Ts...>;
} // namespace

PowerRepeatTask::PowerRepeatTask()
	: util::PeriodicTask<>(Constants::JOINT_POWER_REPEAT_PERIOD,
						   std::bind(&PowerRepeatTask::periodicTask, this)) {}

void PowerRepeatTask::setJointPower(jointid_t id, double power) {
	std::lock_guard lock(_mutex);
	_last_joint_power[id] = power;
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
	if (_last_cmd_vel) {
		if (_last_cmd_vel->first != 0.0 || _last_cmd_vel->second != 0.0) {
			if (_tank) {
				robot::setTankCmdVel(_last_cmd_vel->first, _last_cmd_vel->second);
			} else {
				robot::setCmdVel(_last_cmd_vel->first, _last_cmd_vel->second);
			}
		}
	}
}

CameraStreamTask::CameraStreamTask(websocket::SingleClientWSServer& server)
	: util::AsyncTask<>("MCP_Stream"), _server(server) {}

void CameraStreamTask::openStream(const CameraID& cam, int fps) {
//	std::lock_guard lock(_mutex);
//	_open_streams[cam] = 0;
//	auto it = Constants::video::STREAM_RFS.find(cam);
//	int rf = it != Constants::video::STREAM_RFS.end() ? it->second
//													  : Constants::video::H264_RF_CONSTANT;
//	_camera_encoders[cam] = std::make_shared<video::H264Encoder>(fps, rf);

	if (_open_streams.find(cam) == _open_streams.end()) {
		std::thread([this, cam, fps]() {
			std::lock_guard lock(_mutex);
			auto cfgIt = Constants::CAMERA_CONFIG_PATHS.find(cam);
			// check if we have a config for this camera
			if (cfgIt == Constants::CAMERA_CONFIG_PATHS.end()) {
				LOG_F(WARNING, "No camera configuration found for %s", cam.c_str());
				return;
			}

			// load the config file
			cv::FileStorage configFs(cfgIt->second, cv::FileStorage::READ);
			if (!configFs.isOpened()) {
				LOG_F(ERROR, "Failed to open camera config file %s", cfgIt->second.c_str());
				return;
			}

			bool openCVEnabled = false;
			// check if OpenCV processing is enabled
			// if (!configFs[cam::KEY_OPENCV_ENABLED].empty()) {
			// 	openCVEnabled = static_cast<int>(configFs[cam::KEY_OPENCV_ENABLED]);
			// }

			LOG_F(INFO, "Camera %s OpenCV enabled: %s", cam.c_str(), openCVEnabled ? "true" : "false");
			

			bool opened = false;
			if (!openCVEnabled) {
				if (!configFs[cam::KEY_CAMERA_ID].empty() && !configFs[cam::KEY_FORMAT].empty() &&
					!configFs[cam::KEY_IMAGE_WIDTH].empty() && !configFs[cam::KEY_IMAGE_HEIGHT].empty() &&
					!configFs[cam::KEY_FRAMERATE].empty()) {
					std::string format = static_cast<std::string>(configFs[cam::KEY_FORMAT]);
					std::transform(format.begin(), format.end(), format.begin(),
								   [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
					if (format.find("264") != std::string::npos) {
						cam::CameraStreamProperties streamProps{
							.cameraId = static_cast<int>(configFs[cam::KEY_CAMERA_ID]),
							.format = format,
							.width = static_cast<int>(configFs[cam::KEY_IMAGE_WIDTH]),
							.height = static_cast<int>(configFs[cam::KEY_IMAGE_HEIGHT]),
							.framerate = static_cast<int>(configFs[cam::KEY_FRAMERATE]),
						};
						// try to create H264 passthrough source
						try {
							auto passthrough =
								std::make_unique<cam::H264PassthroughSource>(streamProps);
							_open_streams.insert_or_assign(
								cam, stream_data_t(passthrough_stream_t{std::move(passthrough)}));
							LOG_F(INFO, "Opened H264 pass-through stream for %s", cam.c_str());
							opened = true;
						} catch (const std::exception& e) {
							LOG_F(ERROR, "Failed to initialize H264 pass-through for %s: %s",
								  cam.c_str(), e.what());
						}
					} else {
						LOG_F(INFO,
							  "Camera %s format %s does not provide H264, falling back to CPU "
							  "encoding",
							  cam.c_str(), format.c_str());
					}
				} else {
					LOG_F(WARNING,
						  "Camera %s config missing stream properties; falling back to CPU encoding",
						  cam.c_str());
				}
			}

			if (!opened) {
				auto it = Constants::video::STREAM_RFS.find(cam);
				int rf = (it != Constants::video::STREAM_RFS.end()) ? it->second
																   : Constants::video::H264_RF_CONSTANT;
				auto enc = std::make_shared<video::H264Encoder>(fps, rf);
				auto cam_handle = robot::openCamera(cam);
				if (cam_handle) {
					_open_streams.insert_or_assign(
						cam, stream_data_t(decoded_stream_t{enc, cam_handle}));
					LOG_F(INFO, "Opened CPU-encoded stream for %s", cam.c_str());
				} else {
					LOG_F(ERROR, "Failed to open %s camera for CPU encoding", cam.c_str());
				}
				
			}
		}).detach();
	}
}

void CameraStreamTask::closeStream(const CameraID& cam) {
	std::thread([this, cam]() {
		std::lock_guard lock(_mutex);
		_open_streams.erase(cam);
	}).detach();
}

void CameraStreamTask::task(std::unique_lock<std::mutex>&) {
	while (isRunningInternal()) {
		{
			std::lock_guard lg(_mutex);
			// for all open streams, check if there is a new frame
			for (auto& [cam, stream_data] : _open_streams) {
				std::visit(
					overloaded{
						[this, &stream_data, &cam](decoded_stream_t& decoded) {
							uint32_t frame_num = stream_data.frame_num;
							if (robot::hasNewCameraFrame(cam, frame_num)) {
								auto camDP = robot::readCamera(cam);
								if (camDP) {
									auto data = camDP.getData();
									uint32_t& new_frame_num = data.second;
									cv::Mat frame = data.first;
									stream_data.frame_num = new_frame_num;
									const auto& encoder = decoded.encoder;

									auto data_vector = encoder->encode_frame(frame);
									json msg = {{"type", CAMERA_STREAM_REP_TYPE},
												{"camera", cam},
												{"data", data_vector}};
									_server.sendJSON(Constants::MC_PROTOCOL_NAME, msg);
								}
							}
						},
						[this, &stream_data, &cam](passthrough_stream_t& passthrough) {
							if (!passthrough.source) {
								return;
							}
							std::vector<std::basic_string<uint8_t>> data_vector;
							uint32_t new_frame_num = stream_data.frame_num;
							if (passthrough.source->next(data_vector, new_frame_num) &&
								!data_vector.empty()) {
								stream_data.frame_num = new_frame_num;
								json msg = {{"type", CAMERA_STREAM_REP_TYPE},
											{"camera", cam},
											{"data", data_vector}};
								_server.sendJSON(Constants::MC_PROTOCOL_NAME, msg);
							}
						}},
					stream_data.stream);
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
		double lon = 0, lat = 0, alt = 0;
		if (gps.isValid()) {
			lon = gps.getData().lon;
			lat = gps.getData().lat;
			alt = gps.getData().alt;
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
					{"alt", alt},
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

} // namespace net::mc::tasks
