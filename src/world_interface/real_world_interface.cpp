#include "../CAN/CAN.h"
#include "../CAN/CANMotor.h"
#include "../CAN/CANUtils.h"
#include "../Globals.h"
#include "../Util.h"
#include "../ar/read_landmarks.h"
#include "../camera/Camera.h"
#include "../gps/usb_gps/read_usb_gps.h"
#include "../lidar/read_hokuyo_lidar.h"
#include "../log.h"
#include "../navtypes.h"
#include "kinematic_common_interface.h"
#include "real_world_constants.h"
#include "world_interface.h"

#include <future>
#include <iostream>
#include <map>
#include <set>
#include <vector>

#include <nlohmann/json.hpp>
#include <opencv2/calib3d.hpp>

using nlohmann::json;
using namespace navtypes;
using namespace robot::types;

namespace robot {

const WorldInterface WORLD_INTERFACE = WorldInterface::real;

namespace {
// map that associates camera id to the camera object
std::map<CameraID, std::shared_ptr<cam::Camera>> cameraMap;

std::map<motorid_t, can::motormode_t> motorModeMap;

void ensureMotorMode(motorid_t motor, can::motormode_t mode) {
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
		can::motor::initMotor(serial, invEnc, true, ppjr, TELEM_PERIOD, pid.kP, pid.kI,
							  pid.kD);
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

double setCmdVel(double dtheta, double dx) {
	if (Globals::E_STOP && (dtheta != 0 || dx != 0))
		return 0;

	/* This is the inverse of the formula:
	 *		dx = (right_ground_vel + left_ground_vel) / 2
	 *		dtheta = (right_ground_vel - left_ground_vel) / EFF_WHEEL_BASE
	 */
	double right_ground_vel = dx + EFF_WHEEL_BASE / 2 * dtheta;
	double left_ground_vel = dx - EFF_WHEEL_BASE / 2 * dtheta;
	double right_angular_vel = right_ground_vel / WHEEL_RADIUS;
	double left_angular_vel = left_ground_vel / WHEEL_RADIUS;
	int16_t right_pwm = (int16_t)(right_angular_vel * PWM_FOR_1RAD_PER_SEC);
	int16_t left_pwm = (int16_t)(left_angular_vel * PWM_FOR_1RAD_PER_SEC);
	log(LOG_TRACE, "dtheta %f dx %f right ground %f right angular %f right pwm %d\n", dtheta,
		dx, right_ground_vel, right_angular_vel, right_pwm);
	double scale_down_factor = 1.0;
	if (abs(right_pwm) > MAX_PWM) {
		log(LOG_WARN, "WARNING: requested too-large right PWM %d\n", right_pwm);
		scale_down_factor = abs(right_pwm) / MAX_PWM;
	}
	if (abs(left_pwm) > MAX_PWM) {
		log(LOG_WARN, "WARNING: requested too-large left PWM %d\n", left_pwm);
		double scale_down_factor_left = abs(left_pwm) / MAX_PWM;
		if (scale_down_factor_left > scale_down_factor)
			scale_down_factor = scale_down_factor_left;
	}
	right_pwm = (int16_t)(right_pwm / scale_down_factor);
	left_pwm = (int16_t)(left_pwm / scale_down_factor);
	if (scale_down_factor < 1.0) {
		log(LOG_WARN, "Scaling down cmd_vel by %f to %f %f\n", scale_down_factor,
			dtheta / scale_down_factor, dx / scale_down_factor);
	}

	setCmdVelToIntegrate(dtheta / scale_down_factor, dx / scale_down_factor);

	CANPacket p;
	uint8_t motor_group = 0x04;
	AssemblePWMDirSetPacket(&p, motor_group, DEVICE_SERIAL_MOTOR_CHASSIS_FL, left_pwm);
	can::sendCANPacket(p);
	AssemblePWMDirSetPacket(&p, motor_group, DEVICE_SERIAL_MOTOR_CHASSIS_FR, right_pwm);
	can::sendCANPacket(p);
	AssemblePWMDirSetPacket(&p, motor_group, DEVICE_SERIAL_MOTOR_CHASSIS_BL, left_pwm);
	can::sendCANPacket(p);
	AssemblePWMDirSetPacket(&p, motor_group, DEVICE_SERIAL_MOTOR_CHASSIS_BR, right_pwm);
	can::sendCANPacket(p);

	return scale_down_factor;
}

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

const std::map<motorid_t, double> positive_arm_pwm_scales = {
	{motorid_t::armBase, 0.1831},
	{motorid_t::shoulder, -0.3052},
	{motorid_t::elbow, -0.5},
	{motorid_t::forearm, -0.0916},
	{motorid_t::differentialLeft, 0.0763},
	{motorid_t::differentialRight, -0.0763},
	{motorid_t::hand, 0.2289}};
const std::map<motorid_t, double> negative_arm_pwm_scales = {
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
	ensureMotorMode(motor, can::motormode_t::pwm);
	can::motor::setMotorPower(serial, power);
}

void setMotorPos(robot::types::motorid_t motor, int32_t targetPos) {
	can::deviceserial_t serial = motorSerialIDMap.at(motor);
	ensureMotorMode(motor, can::motormode_t::pid);
	can::motor::setMotorPIDTarget(serial, targetPos);
}

// TODO: implement
void setIndicator(indication_t signal) {}

types::DataPoint<int32_t> getMotorPos(robot::types::motorid_t motor) {
	return can::motor::getMotorPosition(motorSerialIDMap.at(motor));
}

} // namespace robot
