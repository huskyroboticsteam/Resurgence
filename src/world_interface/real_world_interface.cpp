#include "../CAN/CAN.h"
#include "../CAN/CANMotor.h"
#include "../CAN/CANUtils.h"
#include "../Constants.h"
#include "../ardupilot/ArduPilotInterface.h"
#include "../camera/Camera.h"
#include "../camera/CameraConfig.h"
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
  double posScale = 0;
  double negScale = 0;

	// get scales for motor
  try {
    posScale = positive_pwm_scales.at(motor);
    negScale = negative_pwm_scales.at(motor);
  } catch (const std::out_of_range& err) {
    LOG_F(ERROR, "Couldn't find PWM scales for motor 0x%x", static_cast<uint8_t>(motor));
  }

	// create ptr and insert in map
	std::shared_ptr<robot::base_motor> ptr =
		std::make_shared<can_motor>(motor, hasPosSensor, motorSerialIDMap.at(motor),
									motorGroupMap.at(motor), posScale, negScale);
	motor_ptrs.insert({motor, ptr});
}
} // namespace

const kinematics::DiffDriveKinematics& driveKinematics() {
	return drive_kinematics;
}

namespace {
// map that associates camera id to the camera object
std::unordered_map<CameraID, std::weak_ptr<cam::Camera>> cameraMap;

callbackid_t nextCallbackID = 0;
std::unordered_map<callbackid_t, can::callbackid_t> callbackIDMap;

void initMotors() {
	for (const auto& it : motorSerialIDMap) {
		motorid_t motor = it.first;
		auto group = motorGroupMap.at(motor);
		can::motor::initMotor(group, it.second);
		bool hasPosSensor = robot::potMotors.find(motor) != robot::potMotors.end() ||
							robot::encMotors.find(motor) != robot::encMotors.end();
		addMotorMapping(motor, hasPosSensor);
	}

	for (const auto& pot_motor_pair : robot::potMotors) {
		motorid_t motor_id = pot_motor_pair.first;
		potparams_t pot_params = pot_motor_pair.second;

		can::devicegroup_t group = motorGroupMap.at(motor_id);
		can::deviceserial_t serial = motorSerialIDMap.at(motor_id);

		can::motor::initPotentiometer(group, serial, pot_params.mdeg_lo, pot_params.mdeg_hi,
									  pot_params.adc_lo, pot_params.adc_hi, TELEM_PERIOD);
	}

	for (const auto& enc_motor_pair : robot::encMotors) {
		motorid_t motor_id = enc_motor_pair.first;
		encparams_t enc_params = enc_motor_pair.second;

		can::devicegroup_t group = motorGroupMap.at(motor_id);
		can::deviceserial_t serial = motorSerialIDMap.at(motor_id);

		can::motor::initEncoder(group, serial, enc_params.isInverted, true, enc_params.ppjr,
								TELEM_PERIOD);
		can::motor::setLimitSwitchLimits(group, serial, enc_params.limitSwitchLow,
										 enc_params.limitSwitchHigh);
	}

	for (const auto& pair : robot::motorPIDMap) {
		motorid_t motor = pair.first;
		can::devicegroup_t group = motorGroupMap.at(motor);
		can::deviceserial_t serial = motorSerialIDMap.at(motor);
		pidcoef_t pid = motorPIDMap.at(motor);
		can::motor::setMotorPIDConstants(group, serial, pid.kP, pid.kI, pid.kD);
	}
}

std::shared_ptr<cam::Camera> openCamera_(CameraID camID) {
	auto it = cameraMap.find(camID);
	if (it != cameraMap.end()) {
		auto cam = it->second.lock();
		if (cam) { return cam; }
	}
	try {
		// Load camera configuration to get intrinsic and extrinsic parameters
		auto config = cam::readConfigFromFile(Constants::CAMERA_CONFIG_PATHS.at(camID));
		
		auto cam = std::make_shared<cam::Camera>();
		// Extract values from optional before passing to open()
		cam::CameraParams intrinsics = config.intrinsicParams.value_or(cam::CameraParams());
		cv::Mat extrinsics = config.extrinsicParams.value_or(cv::Mat());
		bool success = cam->open(camID, intrinsics, extrinsics);
		if (success) {
			cameraMap[camID] = cam;
			return cam;
		} else {
			LOG_F(ERROR, "Failed to open %s camera", camID.c_str());
		}
	} catch (const std::exception& e) {
		LOG_F(ERROR, "Error opening %s camera:\n%s", camID.c_str(), e.what());
	}

	return nullptr;
}
} // namespace

void world_interface_init(
	std::optional<std::reference_wrapper<net::websocket::SingleClientWSServer>> wsServer,
	bool initOnlyMotors) {
	if (!initOnlyMotors) {
		if (wsServer.has_value()) {
			ardupilot::initArduPilotProtocol(wsServer.value());
		}
	}
	can::initCAN();
	initMotors();

  // Initialize Science Servo Board

  // For now, we can consider the board as a motor and just use it for its serial
}

std::shared_ptr<types::CameraHandle> openCamera(CameraID cameraID) {
	std::shared_ptr<cam::Camera> cam = openCamera_(cameraID);
	return cam ? std::make_shared<types::CameraHandle>(cam) : nullptr;
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
		// return itr->second->hasNext(oldFrameNum);
		auto cam = itr->second.lock();
		if (cam) {
			return cam->hasNext(oldFrameNum);
		} else {
			LOG_F(WARNING, "Cam %s not available", cameraID.c_str());
			return false;
		}
	} else {
		// LOG_F(WARNING, "Invalid camera id: %s", util::to_string(cameraID).c_str());
		return false;
	}
}

DataPoint<CameraFrame> readCamera(CameraID cameraID) {
	auto itr = cameraMap.find(cameraID);
	if (itr != cameraMap.end()) {
		auto cam = itr->second.lock();
		if (!cam) {
			LOG_F(WARNING, "Cam %s not available", cameraID.c_str());
			return DataPoint<CameraFrame>{};
		}
		cv::Mat mat;
		uint32_t frameNum;
		datatime_t time;
		bool success = cam->next(mat, frameNum, time);
		if (success) {
			return DataPoint<CameraFrame>{time, {mat, frameNum}};
		} else {
			return DataPoint<CameraFrame>{};
		}
	} else {
		LOG_F(WARNING, "Invalid camera id: %s", cameraID.c_str());
		return DataPoint<CameraFrame>{};
	}
}

std::optional<cam::CameraParams> getCameraIntrinsicParams(CameraID cameraID) {
	auto itr = cameraMap.find(cameraID);
	if (itr != cameraMap.end()) {
		auto camera = itr->second.lock();
		if (camera) {
			return camera->hasIntrinsicParams() ? camera->getIntrinsicParams()
												: std::optional<cam::CameraParams>{};
		} else {
			LOG_F(WARNING, "Cam %s not available", cameraID.c_str());
			return {};
		}
	} else {
		LOG_F(WARNING, "Invalid camera id: %s", cameraID.c_str());
		return {};
	}
}

std::optional<cv::Mat> getCameraExtrinsicParams(CameraID cameraID) {
	auto itr = cameraMap.find(cameraID);
	if (itr != cameraMap.end()) {
		auto camera = itr->second.lock();
		if (camera) {
			return camera->hasExtrinsicParams() ? camera->getExtrinsicParams()
												: std::optional<cv::Mat>{};
		} else {
			LOG_F(WARNING, "Cam %s not available", cameraID.c_str());
			return {};
		}
	} else {
		LOG_F(WARNING, "Invalid camera id: %s", cameraID.c_str());
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

void setServoPos(robot::types::servoid_t servo, int32_t position) {
  std::shared_ptr<robot::base_motor> servo_board = getMotor(motorid_t::scienceServoBoard);
  auto servo_num = servoid_to_servo_num.find(servo);
  if (servo_num != servoid_to_servo_num.end()) {
  	servo_board->setServoPos(servo_num->second, position);
  }
}

void setRequestedStepperTurnAngle(robot::types::stepperid_t stepper, int16_t angle) {
  std::shared_ptr<robot::base_motor> stepper_board = getMotor(motorid_t::scienceStepperBoard);
  auto stepper_num = stepperid_to_stepper_num.find(stepper);
  if (stepper_num != stepperid_to_stepper_num.end()) {
    stepper_board->setStepperTurnAngle(stepper_num->second, angle);
  }
}

void setActuator(uint8_t value) {
  can::motor::setActuator(can::devicegroup_t::motor, 0x6, value);
}

// TODO: implement
void setIndicator(indication_t signal) {}

callbackid_t addLimitSwitchCallback(
	robot::types::motorid_t motor,
	const std::function<void(robot::types::motorid_t motor,
							 robot::types::DataPoint<LimitSwitchData> limitSwitchData)>&
		callback) {
	auto func = [=](can::devicegroup_t group, can::deviceserial_t serial,
					DataPoint<LimitSwitchData> data) { callback(motor, data); };
	auto id = can::motor::addLimitSwitchCallback(motorGroupMap.at(motor),
												 motorSerialIDMap.at(motor), func);
	auto nextID = nextCallbackID++;
	callbackIDMap.insert({nextID, id});
	return nextID;
}

void removeLimitSwitchCallback(callbackid_t id) {
	return can::motor::removeLimitSwitchCallback(callbackIDMap.at(id));
}

} // namespace robot
