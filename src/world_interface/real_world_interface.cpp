#include "../CAN/CAN.h"
#include "../CAN/CANMotor.h"
#include "../CAN/CANUtils.h"
#include "../Constants.h"
#include "../Globals.h"
#include "../Util.h"
#include "../ar/read_landmarks.h"
#include "../camera/Camera.h"
#include "../gps/usb_gps/read_usb_gps.h"
#include "../lidar/read_hokuyo_lidar.h"
#include "../log.h"
#include "../navtypes.h"
#include "real_world_constants.h"
#include "world_interface.h"

#include <future>
#include <iostream>
#include <unordered_map>
#include <vector>

#include <opencv2/calib3d.hpp>

using nlohmann::json;
using namespace navtypes;
using namespace robot::types;
using can::motor::motormode_t;

namespace robot {

extern const WorldInterface WORLD_INTERFACE = WorldInterface::real;

namespace {
DiffDriveKinematics drive_kinematics(Constants::EFF_WHEEL_BASE);
DiffWristKinematics wrist_kinematics;
} // namespace

const DiffDriveKinematics& driveKinematics() {
	return drive_kinematics;
}

const DiffWristKinematics& wristKinematics() {
	return wrist_kinematics;
}

namespace {
// map that associates camera id to the camera object
std::unordered_map<CameraID, std::shared_ptr<cam::Camera>> cameraMap;

std::unordered_map<motorid_t, motormode_t> motorModeMap;

callbackid_t nextCallbackID = 0;
std::unordered_map<callbackid_t, can::callbackid_t> callbackIDMap;

void ensureMotorMode(motorid_t motor, motormode_t mode) {
	auto entry = motorModeMap.find(motor);
	if (entry == motorModeMap.end()) {
		motorModeMap.insert(std::make_pair(motor, mode));
	} else if (entry->second != mode) {
		entry->second = mode;
		can::motor::setMotorMode(motorSerialIDMap.at(motor), mode);
	}
}

void initMotors() {
	can::motor::initMotor(motorSerialIDMap.at(motorid_t::frontLeftWheel));
	can::motor::initMotor(motorSerialIDMap.at(motorid_t::frontRightWheel));
	can::motor::initMotor(motorSerialIDMap.at(motorid_t::rearLeftwheel));
	can::motor::initMotor(motorSerialIDMap.at(motorid_t::rearRightWheel));

	for (motorid_t motor : pidMotors) {
		can::deviceserial_t serial = motorSerialIDMap.at(motor);
		bool invEnc = motorEncInvMap.at(motor);
		pidcoef_t pid = motorPIDMap.at(motor);
		uint32_t ppjr = motorPulsesPerJointRevMap.at(motor);
		can::motor::initMotor(serial);
		can::motor::initEncoder(serial, invEnc, true, ppjr, TELEM_PERIOD);
		can::motor::setMotorPIDConstants(serial, pid.kP, pid.kI, pid.kD);
	}

	can::motor::initMotor(motorSerialIDMap.at(motorid_t::hand));
}

void setupCameras() {
	try {
		auto arCam = std::make_shared<cam::Camera>();
		arCam->openFromConfigFile(Constants::AR_CAMERA_CONFIG_PATH);
		cameraMap[Constants::AR_CAMERA_ID] = arCam;
	} catch (const std::exception& e) {
		log(LOG_ERROR, "Error opening camera with id %s:\n%s\n", Constants::AR_CAMERA_ID,
			e.what());
	}

	// Set up the rest of the cameras here
}
} // namespace

void world_interface_init() {
	setupCameras();

	bool gps_success = gps::usb::startGPSThread();
	bool lidar_success = lidar::initializeLidar();
	bool landmark_success = AR::initializeLandmarkDetection();

	can::initCAN();
	initMotors();
}

bool hasNewCameraFrame(CameraID cameraID, uint32_t oldFrameNum) {
	auto itr = cameraMap.find(cameraID);
	if (itr != cameraMap.end()) {
		return itr->second->hasNext(oldFrameNum);
	} else {
		log(LOG_WARN, "Invalid camera id: %s\n", cameraID);
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
		log(LOG_WARN, "Invalid camera id: %s\n", cameraID);
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
		log(LOG_WARN, "Invalid camera id: %s\n", cameraID);
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
		log(LOG_WARN, "Invalid camera id: %s\n", cameraID);
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
	return AR::readLandmarks();
}

DataPoint<points_t> readLidarScan() {
	return lidar::readLidar();
}

DataPoint<double> readIMUHeading() {
	return {}; // TODO: interface with IMU
}

DataPoint<pose_t> readVisualOdomVel() {
	return DataPoint<pose_t>{};
}

URCLeg getLeg(int index) {
	return URCLeg{0, -1, {0., 0., 0.}};
}

const std::unordered_map<motorid_t, double> positive_arm_pwm_scales = {
	{motorid_t::armBase, 0.1831},
	{motorid_t::shoulder, -0.3052},
	{motorid_t::elbow, -0.5},
	{motorid_t::forearm, -0.0916},
	{motorid_t::differentialLeft, 0.0763},
	{motorid_t::differentialRight, -0.0763},
	{motorid_t::hand, 0.2289}};
const std::unordered_map<motorid_t, double> negative_arm_pwm_scales = {
	{motorid_t::armBase, 0.1831},
	{motorid_t::shoulder, -0.2136},
	{motorid_t::elbow, -0.2747},
	{motorid_t::forearm, -0.0916},
	{motorid_t::differentialLeft, 0.0763},
	{motorid_t::differentialRight, -0.1526},
	{motorid_t::hand, 0.2289}};

template <typename T> int getIndex(const std::vector<T>& vec, const T& val) {
	auto itr = std::find(vec.begin(), vec.end(), val);
	return itr == vec.end() ? -1 : itr - vec.begin();
}

void setMotorPower(robot::types::motorid_t motor, double power) {
	can::deviceserial_t serial = motorSerialIDMap.at(motor);
	auto& scaleMap = power < 0 ? negative_arm_pwm_scales : positive_arm_pwm_scales;
	auto entry = scaleMap.find(motor);
	if (entry != scaleMap.end()) {
		power *= entry->second;
	}
	ensureMotorMode(motor, motormode_t::pwm);
	can::motor::setMotorPower(serial, power);
}

void setMotorPos(robot::types::motorid_t motor, int32_t targetPos) {
	can::deviceserial_t serial = motorSerialIDMap.at(motor);
	ensureMotorMode(motor, motormode_t::pid);
	can::motor::setMotorPIDTarget(serial, targetPos);
}

// TODO: implement
void setIndicator(indication_t signal) {}

types::DataPoint<int32_t> getMotorPos(robot::types::motorid_t motor) {
	return can::motor::getMotorPosition(motorSerialIDMap.at(motor));
}

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
