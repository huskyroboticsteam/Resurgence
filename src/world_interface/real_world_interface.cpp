#include "../CAN/CAN.h"
#include "../CAN/CANMotor.h"
#include "../CAN/CANUtils.h"
#include "../Constants.h"
#include "../ardupilot/ArduPilotInterface.h"
#include "../camera/Camera.h"
#include "../control_interface.h"
#include "../gps/usb_gps/read_usb_gps.h"
#include "../navtypes.h"
#include "../utils/core.h"
#include "motor/can_motor.h"
#include "real_world_constants.h"
#include "world_interface.h"

#include <future>
#include <iostream>
#include <loguru.hpp>
#include <unordered_map>
#include <vector>

#include <opencv2/calib3d.hpp>

using nlohmann::json;
using namespace navtypes;
using namespace robot::types;
using can::motor::motormode_t;

namespace robot {

extern const WorldInterface WORLD_INTERFACE = WorldInterface::real;

// A mapping of (motor_id, shared pointer to object of the motor)
std::unordered_map<robot::types::motorid_t, std::shared_ptr<robot::base_motor>> motor_ptrs;

namespace {
kinematics::DiffDriveKinematics drive_kinematics(Constants::EFF_WHEEL_BASE);
bool is_emergency_stopped = false;

void addMotorMapping(motorid_t motor, bool hasPosSensor) {
	// get scales for motor
	double posScale = positive_pwm_scales.at(motor);
	double negScale = negative_pwm_scales.at(motor);

	// create ptr and insert in map
	std::shared_ptr<robot::base_motor> ptr = std::make_shared<can_motor>(
		motor, hasPosSensor, motorSerialIDMap.at(motor), posScale, negScale);
	motor_ptrs.insert({motor, ptr});
}
} // namespace

const kinematics::DiffDriveKinematics& driveKinematics() {
	return drive_kinematics;
}

namespace {
// map that associates camera id to the camera object
std::unordered_map<CameraID, std::shared_ptr<cam::Camera>> cameraMap;

callbackid_t nextCallbackID = 0;
std::unordered_map<callbackid_t, can::callbackid_t> callbackIDMap;

void initMotors() {
	verifyAllMotorsConnected();
	for (const auto& it : motorSerialIDMap) {
		can::motor::initMotor(it.second);
		bool hasPosSensor = robot::potMotors.find(it.first) != robot::potMotors.end() ||
							robot::encMotors.find(it.first) != robot::encMotors.end();
		addMotorMapping(it.first, hasPosSensor);
	}

	for (const auto& pot_motor_pair : robot::potMotors) {
		motorid_t motor_id = pot_motor_pair.first;
		potparams_t pot_params = pot_motor_pair.second;

		can::deviceserial_t serial = motorSerialIDMap.at(motor_id);

		can::motor::initPotentiometer(serial, pot_params.mdeg_lo, pot_params.mdeg_hi,
									  pot_params.adc_lo, pot_params.adc_hi, TELEM_PERIOD);
	}

	for (const auto& enc_motor_pair : robot::encMotors) {
		motorid_t motor_id = enc_motor_pair.first;
		encparams_t enc_params = enc_motor_pair.second;

		can::deviceserial_t serial = motorSerialIDMap.at(motor_id);

		can::motor::initEncoder(serial, enc_params.isInverted, true, enc_params.ppjr,
								TELEM_PERIOD);
		can::motor::setLimitSwitchLimits(serial, enc_params.limitSwitchLow,
										 enc_params.limitSwitchHigh);
	}

	for (const auto& pair : robot::motorPIDMap) {
		motorid_t motor = pair.first;
		can::deviceserial_t serial = motorSerialIDMap.at(motor);
		pidcoef_t pid = motorPIDMap.at(motor);
		can::motor::setMotorPIDConstants(serial, pid.kP, pid.kI, pid.kD);
	}
}

void openCamera(CameraID camID, const char* cameraPath) {
	try {
		auto cam = std::make_shared<cam::Camera>();
		bool success = cam->openFromConfigFile(cameraPath);
		if (success) {
			cameraMap[camID] = cam;
		} else {
			LOG_F(ERROR, "Failed to open camera with id %s", util::to_string(camID).c_str());
		}
	} catch (const std::exception& e) {
		LOG_F(ERROR, "Error opening camera with id %s:\n%s", util::to_string(camID).c_str(),
			  e.what());
	}
}

void setupCameras() {
	openCamera(Constants::MAST_CAMERA_ID, Constants::MAST_CAMERA_CONFIG_PATH);
	openCamera(Constants::FOREARM_CAMERA_ID, Constants::FOREARM_CAMERA_CONFIG_PATH);
	openCamera(Constants::HAND_CAMERA_ID, Constants::HAND_CAMERA_CONFIG_PATH);
}
} // namespace

void world_interface_init(
	std::optional<std::reference_wrapper<net::websocket::SingleClientWSServer>> wsServer,
	bool initOnlyMotors) {
	if (!initOnlyMotors) {
		setupCameras();
		if (wsServer.has_value()) {
			ardupilot::initArduPilotProtocol(wsServer.value());
		}
	}
	can::initCAN();
	initMotors();
}

std::shared_ptr<robot::base_motor> getMotor(robot::types::motorid_t motor) {
	auto itr = motor_ptrs.find(motor);

	if (itr == motor_ptrs.end()) {
		// motor id not in map
		LOG_F(ERROR, "getMotor(): Unknown motor %d", static_cast<int>(motor));
		return nullptr;
	} else {
		// return motor object pointer
		return itr->second;
	}
}

void emergencyStop() {
	can::motor::emergencyStopMotors();
	is_emergency_stopped = true;
}

bool isEmergencyStopped() {
	return is_emergency_stopped;
}

std::unordered_set<CameraID> getCameras() {
	return util::keySet(cameraMap);
}

bool hasNewCameraFrame(CameraID cameraID, uint32_t oldFrameNum) {
	auto itr = cameraMap.find(cameraID);
	if (itr != cameraMap.end()) {
		return itr->second->hasNext(oldFrameNum);
	} else {
		LOG_F(WARNING, "Invalid camera id: %s", util::to_string(cameraID).c_str());
		return false;
	}
}

DataPoint<CameraFrame> readCamera(CameraID cameraID) {
	auto itr = cameraMap.find(cameraID);
	if (itr != cameraMap.end()) {
		cv::Mat mat;
		uint32_t frameNum;
		datatime_t time;
		bool success = itr->second->next(mat, frameNum, time);
		if (success) {
			return DataPoint<CameraFrame>{time, {mat, frameNum}};
		} else {
			return DataPoint<CameraFrame>{};
		}
	} else {
		LOG_F(WARNING, "Invalid camera id: %s", util::to_string(cameraID).c_str());
		return DataPoint<CameraFrame>{};
	}
}

std::optional<cam::CameraParams> getCameraIntrinsicParams(CameraID cameraID) {
	auto itr = cameraMap.find(cameraID);
	if (itr != cameraMap.end()) {
		auto camera = itr->second;
		return camera->hasIntrinsicParams() ? camera->getIntrinsicParams()
											: std::optional<cam::CameraParams>{};
	} else {
		LOG_F(WARNING, "Invalid camera id: %s", util::to_string(cameraID).c_str());
		return {};
	}
}

std::optional<cv::Mat> getCameraExtrinsicParams(CameraID cameraID) {
	auto itr = cameraMap.find(cameraID);
	if (itr != cameraMap.end()) {
		auto camera = itr->second;
		return camera->hasExtrinsicParams() ? camera->getExtrinsicParams()
											: std::optional<cv::Mat>{};
	} else {
		LOG_F(WARNING, "Invalid camera id: %s", util::to_string(cameraID).c_str());
		return {};
	}
}

// Distance between left and right wheels.
constexpr double WHEEL_BASE = 0.66;
// Effective distance between wheels. Tweaked so that actual rover angular rate
// roughly matches the commanded angular rate.
constexpr double EFF_WHEEL_BASE = 1.40;

constexpr double WHEEL_RADIUS = 0.15;		  // Eyeballed
constexpr double PWM_FOR_1RAD_PER_SEC = 5000; // Eyeballed
// This is a bit on the conservative side, but we heard an ominous popping sound at 20000.
constexpr double MAX_PWM = 20000;

DataPoint<pose_t> getTruePose() {
	return {};
}

landmarks_t readLandmarks() {
	return {};
}

template <typename T>
int getIndex(const std::vector<T>& vec, const T& val) {
	auto itr = std::find(vec.begin(), vec.end(), val);
	return itr == vec.end() ? -1 : itr - vec.begin();
}

void setMotorPower(robot::types::motorid_t motor, double power) {
	std::shared_ptr<robot::base_motor> motor_ptr = getMotor(motor);
	motor_ptr->setMotorPower(power);
}

void setMotorPos(robot::types::motorid_t motor, int32_t targetPos) {
	std::shared_ptr<robot::base_motor> motor_ptr = getMotor(motor);
	motor_ptr->setMotorPos(targetPos);
}

types::DataPoint<int32_t> getMotorPos(robot::types::motorid_t motor) {
	std::shared_ptr<robot::base_motor> motor_ptr = getMotor(motor);
	return motor_ptr->getMotorPos();
}

void setMotorVel(robot::types::motorid_t motor, int32_t targetVel) {
	std::shared_ptr<robot::base_motor> motor_ptr = getMotor(motor);
	motor_ptr->setMotorVel(targetVel);
}

// TODO: implement
void setIndicator(indication_t signal) {}

callbackid_t addLimitSwitchCallback(
	robot::types::motorid_t motor,
	const std::function<void(robot::types::motorid_t motor,
							 robot::types::DataPoint<LimitSwitchData> limitSwitchData)>&
		callback) {
	auto func = [=](can::deviceserial_t serial, DataPoint<LimitSwitchData> data) {
		callback(motor, data);
	};
	auto id = can::motor::addLimitSwitchCallback(motorSerialIDMap.at(motor), func);
	auto nextID = nextCallbackID++;
	callbackIDMap.insert({nextID, id});
	return nextID;
}

void removeLimitSwitchCallback(callbackid_t id) {
	return can::motor::removeLimitSwitchCallback(callbackIDMap.at(id));
}

} // namespace robot
