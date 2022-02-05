#include "../Constants.h"
#include "../Globals.h"
#include "../Networking/websocket/WebSocketProtocol.h"
#include "../ar/read_landmarks.h"
#include "../camera/Camera.h"
#include "../kinematics/DiffDriveKinematics.h"
#include "../log.h"
#include "kinematic_common_interface.h"
#include "world_interface.h"

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <shared_mutex>
#include <string>

#include <Eigen/Geometry>
#include <nlohmann/json.hpp>
#include <opencv2/core.hpp>

using nlohmann::json;

namespace {
constexpr double MAX_WHEEL_VEL =
	Constants::WHEEL_RADIUS * Constants::MAX_DRIVE_PWM / Constants::PWM_PER_RAD_PER_SEC;
const std::string PROTOCOL_PATH("/simulator");

std::map<std::string, std::mutex> mutexMap;

DiffDriveKinematics kinematics(Constants::EFF_WHEEL_BASE);
DataPoint<double> lastHeading;
DataPoint<gpscoords_t> lastGPS;
DataPoint<points_t> lastLidar;
std::map<CameraID, DataPoint<CameraFrame>> cameraFrameMap;
std::shared_mutex cameraFrameMapMutex;

std::condition_variable connectionCV;
std::mutex connectionMutex;
bool simConnected = false;

void sendJSON(const json& obj) {
	Globals::websocketServer.sendJSON(PROTOCOL_PATH, obj);
}

void initCameras() {
	json msg = {{"type", "simCameraStreamOpenRequest"},
				{"camera", Constants::AR_CAMERA_ID},
				{"fps", 20},
				{"width", 640},
				{"height", 480}};
	sendJSON(msg);
}

void handleGPS(json msg) {
	gpscoords_t coords{msg["latitude"], msg["longitude"]};
	std::lock_guard<std::mutex> guard(mutexMap["gps"]);
	lastGPS = {coords};
}

void handleLidar(json msg) {
	points_t lidar;
	datatime_t now = dataclock::now();
	const auto& arr = msg["points"];
	for (const auto& point : arr) {
		double r = point["r"];
		double theta = point["theta"];
		lidar.push_back({r * std::cos(theta), r * std::sin(theta), 1});
	}
	std::lock_guard<std::mutex> guard(mutexMap["lidar"]);
	lastLidar = {now, lidar};
}

void handleIMU(json msg) {
	double qx = msg["x"];
	double qy = msg["y"];
	double qz = msg["z"];
	double qw = msg["w"];

	// see what +x gets transformed to by this rotation
	Eigen::Quaterniond quat(qw, qx, qy, qz);
	quat.normalize();
	Eigen::Matrix3d rotMat = quat.toRotationMatrix();
	Eigen::Vector3d transformedX = rotMat * Eigen::Vector3d::UnitX();
	// flatten to xy-plane
	transformedX(2) = 0;
	// recover heading
	double heading = std::atan2(transformedX(1), transformedX(0));
	std::lock_guard<std::mutex> guard(mutexMap["imu"]);
	lastHeading = {heading};
}

void handleCamFrame(json msg) {
	// TODO: deserialize b64 -> cv::Mat
}

void handleMotorStatus(json msg) {
	// TODO: forward to mission control
}

void clientConnected() {
	log(LOG_INFO, "Simulator connected!\n");
	{
		std::lock_guard<std::mutex> lock(connectionMutex);
		simConnected = true;
	}
	connectionCV.notify_one();
}

void clientDisconnected() {
	log(LOG_ERROR, "ERROR: Simulator disconnected! World Interface disconnected!\n");
}

void initSimServer() {
	websocket::WebSocketProtocol protocol(PROTOCOL_PATH);
	protocol.addMessageHandler("simImuOrientationReport", handleIMU);
	protocol.addMessageHandler("simLidarReport", handleLidar);
	protocol.addMessageHandler("simGpsPositionReport", handleGPS);
	protocol.addMessageHandler("simCameraStreamReport", handleCamFrame);
	protocol.addMessageHandler("simMotorStatusReport", handleMotorStatus);
	protocol.addConnectionHandler(clientConnected);
	protocol.addDisconnectionHandler(clientDisconnected);

	Globals::websocketServer.addProtocol(protocol);

	{
		// wait for simulator to connect
		log(LOG_INFO, "Waiting for simulator connection...\n");
		std::unique_lock<std::mutex> lock(connectionMutex);
		connectionCV.wait(lock, [] { return simConnected; });
	}

	initCameras();
}

void initMutexMap() {
	mutexMap["imu"];
	mutexMap["gps"];
	mutexMap["lidar"];
}

} // namespace

void world_interface_init() {
	initMutexMap();
	initSimServer();
	initCameras();

	AR::initializeLandmarkDetection();
}

// TODO: implement camera methods

bool hasNewCameraFrame(CameraID cameraID, uint32_t oldFrameNum) {
	return {};
}

DataPoint<CameraFrame> readCamera(CameraID cameraID) {
	return {};
}

std::optional<cam::CameraParams> getCameraIntrinsicParams(CameraID cameraID) {
	return {};
}

std::optional<cv::Mat> getCameraExtrinsicParams(CameraID cameraID) {
	return {};
}

double setCmdVel(double dtheta, double dx) {
	if (Globals::E_STOP && (dtheta != 0 || dx != 0)) {
		return 0;
	}

	wheelvel_t wheelVels = kinematics.robotVelToWheelVel(dx, dtheta);
	double lPWM = wheelVels.lVel / MAX_WHEEL_VEL;
	double rPWM = wheelVels.rVel / MAX_WHEEL_VEL;
	double maxAbsPWM = std::max(std::abs(lPWM), std::abs(rPWM));
	if (maxAbsPWM > 1) {
		lPWM /= maxAbsPWM;
		rPWM /= maxAbsPWM;
	}

	setCmdVelToIntegrate(wheelVels);
	setMotorPWM("frontLeftWheel", lPWM);
	setMotorPWM("rearLeftWheel", lPWM);
	setMotorPWM("frontRightWheel", rPWM);
	setMotorPWM("rearRightWheel", rPWM);

	return maxAbsPWM > MAX_WHEEL_VEL ? maxAbsPWM : 1.0;
}

landmarks_t readLandmarks() {
	return AR::readLandmarks();
}

DataPoint<points_t> readLidarScan() {
	std::lock_guard<std::mutex> guard(mutexMap["lidar"]);
	return lastLidar;
}

DataPoint<pose_t> readVisualOdomVel() {
	return DataPoint<pose_t>{};
}

DataPoint<gpscoords_t> gps::readGPSCoords() {
	std::lock_guard<std::mutex> guard(mutexMap["gps"]);
	return lastGPS;
}

DataPoint<double> readIMUHeading() {
	std::lock_guard<std::mutex> guard(mutexMap["imu"]);
	return lastHeading;
}

URCLeg getLeg(int index) {
	return URCLeg{0, -1, {0., 0., 0.}};
}

void setMotorPWM(const std::string& motor, double normalizedPWM) {
	json msg = {{"type", "simMotorPowerRequest"}, {"motor", motor}, {"power", normalizedPWM}};
	sendJSON(msg);
}

void setMotorPos(const std::string& motor, int32_t targetPos) {
	json msg = {
		{"type", "simMotorPositionRequest"}, {"motor", motor}, {"position", targetPos}};
	sendJSON(msg);
}

void setIndicator(indication_t signal) {
	if (signal == indication_t::arrivedAtDest) {
		log(LOG_INFO, "Robot arrived at destination!\n");
	}
}
