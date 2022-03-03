#include "../Globals.h"
#include "../Networking/CANUtils.h"
#include "../Networking/ParseCAN.h"
#include "../Networking/motor_interface.h"
#include "../Util.h"
#include "../ar/read_landmarks.h"
#include "../camera/Camera.h"
#include "../gps/usb_gps/read_usb_gps.h"
#include "../lidar/read_hokuyo_lidar.h"
#include "../log.h"
#include "../navtypes.h"
#include "kinematic_common_interface.h"
#include "world_interface.h"

#include <future>
#include <iostream>
#include <unordered_map>
#include <unordered_set>

#include <nlohmann/json.hpp>
#include <opencv2/calib3d.hpp>

extern "C" {
#include "../HindsightCAN/CANMotorUnit.h"
}

using nlohmann::json;
using namespace navtypes;

const WorldInterface WORLD_INTERFACE = WorldInterface::real;

// map that associates camera id to the camera object
static std::unordered_map<CameraID, std::shared_ptr<cam::Camera>> cameraMap;

void setupCameras() {
	try {
		auto arCam = std::make_shared<cam::Camera>();
		arCam->openFromConfigFile(Constants::AR_CAMERA_CONFIG_PATH);
		cameraMap[Constants::AR_CAMERA_ID] = arCam;
	} catch (const std::exception& e) {
		log(LOG_ERROR, "Error opening camera with id %s:\n%s\n",
			Constants::AR_CAMERA_ID.c_str(), e.what());
	}

	// Set up the rest of the cameras here
}

void world_interface_init() {
	setupCameras();

	bool gps_success = gps::usb::startGPSThread();
	bool lidar_success = lidar::initializeLidar();
	bool landmark_success = AR::initializeLandmarkDetection();
}

std::unordered_set<CameraID> getCameras() {
	return util::keySet(cameraMap);
}

bool hasNewCameraFrame(CameraID cameraID, uint32_t oldFrameNum) {
	auto itr = cameraMap.find(cameraID);
	if (itr != cameraMap.end()) {
		return itr->second->hasNext(oldFrameNum);
	} else {
		log(LOG_WARN, "Invalid camera id: %s\n", cameraID.c_str());
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
		log(LOG_WARN, "Invalid camera id: %s\n", cameraID.c_str());
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
		log(LOG_WARN, "Invalid camera id: %s\n", cameraID.c_str());
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
		log(LOG_WARN, "Invalid camera id: %s\n", cameraID.c_str());
		return {};
	}
}

double setCmdVel(double dtheta, double dx) {
	if (Globals::E_STOP && (dtheta != 0 || dx != 0))
		return 0;

	/* This is the inverse of the formula:
	 *		dx = (right_ground_vel + left_ground_vel) / 2
	 *		dtheta = (right_ground_vel - left_ground_vel) / EFF_WHEEL_BASE
	 */
	double right_ground_vel = dx + Constants::EFF_WHEEL_BASE / 2 * dtheta;
	double left_ground_vel = dx - Constants::EFF_WHEEL_BASE / 2 * dtheta;
	double right_angular_vel = right_ground_vel / Constants::WHEEL_RADIUS;
	double left_angular_vel = left_ground_vel / Constants::WHEEL_RADIUS;
	int16_t right_pwm = (int16_t)(right_angular_vel * Constants::PWM_PER_RAD_PER_SEC);
	int16_t left_pwm = (int16_t)(left_angular_vel * Constants::PWM_PER_RAD_PER_SEC);
	log(LOG_TRACE, "dtheta %f dx %f right ground %f right angular %f right pwm %d\n", dtheta,
		dx, right_ground_vel, right_angular_vel, right_pwm);
	double scale_down_factor = 1.0;
	if (abs(right_pwm) > Constants::MAX_DRIVE_PWM) {
		log(LOG_WARN, "WARNING: requested too-large right PWM %d\n", right_pwm);
		scale_down_factor = abs(right_pwm) / Constants::MAX_DRIVE_PWM;
	}
	if (abs(left_pwm) > Constants::MAX_DRIVE_PWM) {
		log(LOG_WARN, "WARNING: requested too-large left PWM %d\n", left_pwm);
		double scale_down_factor_left = abs(left_pwm) / Constants::MAX_DRIVE_PWM;
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
	sendCANPacket(p);
	AssemblePWMDirSetPacket(&p, motor_group, DEVICE_SERIAL_MOTOR_CHASSIS_FR, right_pwm);
	sendCANPacket(p);
	AssemblePWMDirSetPacket(&p, motor_group, DEVICE_SERIAL_MOTOR_CHASSIS_BL, left_pwm);
	sendCANPacket(p);
	AssemblePWMDirSetPacket(&p, motor_group, DEVICE_SERIAL_MOTOR_CHASSIS_BR, right_pwm);
	sendCANPacket(p);

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

const std::map<std::string, double> positive_arm_pwm_scales = {
	{"armBase", 12000}, {"shoulder", -20000},		{"elbow", -32768},
	{"forearm", -6000}, {"differentialLeft", 5000}, {"differentialRight", -5000},
	{"hand", 15000}};
const std::map<std::string, double> negative_arm_pwm_scales = {
	{"armBase", 12000}, {"shoulder", -14000},		{"elbow", -18000},
	{"forearm", -6000}, {"differentialLeft", 5000}, {"differentialRight", -10000},
	{"hand", 15000}};
const std::map<std::string, double> incremental_pid_scales = {
	{"armBase", M_PI / 8}, // TODO: Check signs
	{"shoulder", -M_PI / 8},
	{"elbow", -M_PI / 8},
	{"forearm", 0}, // We haven't implemented PID on these motors yet
	{"differentialLeft", 0},
	{"differentialRight", 0},
	{"hand", 0}};

template <typename T> int getIndex(const std::vector<T>& vec, const T& val) {
	auto itr = std::find(vec.begin(), vec.end(), val);
	return itr == vec.end() ? -1 : itr - vec.begin();
}

void setMotorPWM(const std::string& motor, double normalizedPWM) {
	CANPacket p;
	auto& scale_map = (normalizedPWM > 0 ? positive_arm_pwm_scales : negative_arm_pwm_scales);
	double pwm = normalizedPWM * scale_map.at(motor);
	int motor_serial = getIndex(motor_group, motor);
	AssemblePWMDirSetPacket(&p, DEVICE_GROUP_MOTOR_CONTROL, motor_serial, pwm);
	sendCANPacket(p);
}

void setMotorPos(const std::string& motor, int32_t targetPos) {
	CANPacket p;
	int motor_serial = getIndex(motor_group, motor);
	AssemblePIDTargetSetPacket(&p, DEVICE_GROUP_MOTOR_CONTROL, motor_serial, targetPos);
	sendCANPacket(p);
}

// TODO: implement
void setIndicator(indication_t signal) {}
