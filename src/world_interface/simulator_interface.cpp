#include "../Constants.h"
#include "../Globals.h"
#include "../Networking/websocket/WebSocketProtocol.h"
#include "../Util.h"
#include "../ar/read_landmarks.h"
#include "../base64/base64_img.h"
#include "../camera/Camera.h"
#include "../camera/CameraConfig.h"
#include "../kinematics/DiffDriveKinematics.h"
#include "../log.h"
#include "../navtypes.h"
#include "kinematic_common_interface.h"
#include "world_interface.h"

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <shared_mutex>
#include <string>

#include <nlohmann/json.hpp>
#include <opencv2/core.hpp>

using nlohmann::json;
using namespace navtypes;
using namespace robot::types;

namespace {
const std::string PROTOCOL_PATH("/simulator");
const DiffDriveKinematics kinematics(Constants::EFF_WHEEL_BASE);
const std::map<motorid_t, std::string> motorNameMap = {
	{motorid_t::frontLeftWheel, "frontLeftWheel"},
	{motorid_t::frontRightWheel, "frontRightWheel"},
	{motorid_t::rearLeftwheel, "rearLeftwheel"},
	{motorid_t::rearRightWheel, "rearRightWheel"},
	{motorid_t::armBase, "armBase"},
	{motorid_t::shoulder, "shoulder"},
	{motorid_t::elbow, "elbow"},
	{motorid_t::forearm, "forearm"},
	{motorid_t::differentialRight, "differentialRight"},
	{motorid_t::differentialLeft, "differentialLeft"},
	{motorid_t::hand, "hand"}};

DataPoint<double> lastHeading;
std::mutex headingMutex;

DataPoint<gpscoords_t> lastGPS;
std::mutex gpsMutex;

DataPoint<points_t> lastLidar;
std::mutex lidarMutex;

DataPoint<pose_t> lastTruePose;
std::mutex truePoseMutex;

// stores the last camera frame for each camera
std::map<CameraID, DataPoint<CameraFrame>> cameraFrameMap;
// stores the index of the last camera frame for each camera
// cameraFrameMap is not guaranteed to have valid data points,
// but this one is guaranteed to have indices, if there are any
std::map<CameraID, uint32_t> cameraLastFrameIdxMap;
std::shared_mutex cameraFrameMapMutex; // protects both of the above maps
// not modified after startup, no need to synchronize
std::map<CameraID, cam::CameraConfig> cameraConfigMap;

std::condition_variable connectionCV;
std::mutex connectionMutex;
bool simConnected = false;

void sendJSON(const json& obj) {
	Globals::websocketServer.sendJSON(PROTOCOL_PATH, obj);
}

void initCameras() {
	auto cfg = cam::readConfigFromFile(Constants::AR_CAMERA_CONFIG_PATH);
	cameraConfigMap[Constants::AR_CAMERA_ID] = cfg;

	json msg = {{"type", "simCameraStreamOpenRequest"},
				{"camera", Constants::AR_CAMERA_ID},
				{"fps", 20},
				{"width", 640},
				{"height", 480}};
	sendJSON(msg);
}

void handleGPS(json msg) {
	gpscoords_t coords{msg["latitude"], msg["longitude"]};
	std::lock_guard<std::mutex> guard(gpsMutex);
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
	std::lock_guard<std::mutex> guard(lidarMutex);
	lastLidar = {now, lidar};
}

void handleIMU(json msg) {
	double qx = msg["x"];
	double qy = msg["y"];
	double qz = msg["z"];
	double qw = msg["w"];

	double heading = util::quatToHeading(qw, qx, qy, qz);
	std::lock_guard<std::mutex> guard(headingMutex);
	lastHeading = {heading};
}

void handleCamFrame(json msg) {
	std::string cam = msg["camera"];
	std::string b64 = msg["data"];
	cv::Mat mat = base64::decodeMat(b64);

	// acquire exclusive lock
	std::lock_guard<std::shared_mutex> lock(cameraFrameMapMutex);
	auto entry = cameraLastFrameIdxMap.find(cam);
	uint32_t idx = 0;
	if (entry != cameraLastFrameIdxMap.end()) {
		idx = entry->second + 1;
	}

	CameraFrame cf = {mat, idx};
	DataPoint<CameraFrame> df(cf);
	cameraFrameMap[cam] = df;
	cameraLastFrameIdxMap[cam] = idx;
}

void handleMotorStatus(json msg) {
	// TODO: forward to mission control
}

void handleTruePose(json msg) {
	auto pos = msg["position"];
	auto rot = msg["rotation"];
	double qw = rot["w"];
	double qx = rot["x"];
	double qy = rot["y"];
	double qz = rot["z"];
	double heading = util::quatToHeading(qw, qx, qy, qz);

	pose_t pose = {pos["x"], pos["y"], heading};
	std::lock_guard<std::mutex> lock(truePoseMutex);
	lastTruePose = {pose};
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
	protocol.addMessageHandler("simRoverTruePoseReport", handleTruePose);
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

} // namespace

namespace robot {

const WorldInterface WORLD_INTERFACE = WorldInterface::sim3d;

void world_interface_init() {
	initSimServer();
	initCameras();

	AR::initializeLandmarkDetection();
}

bool hasNewCameraFrame(CameraID cameraID, uint32_t oldFrameNum) {
	// acquire shared lock
	std::shared_lock<std::shared_mutex> lock(cameraFrameMapMutex);
	auto cfEntry = cameraFrameMap.find(cameraID);
	if (cfEntry == cameraFrameMap.end() || !cfEntry->second.isValid()) {
		return false;
	}
	return cfEntry->second.getData().second != oldFrameNum;
}

DataPoint<CameraFrame> readCamera(CameraID cameraID) {
	// acquire shared lock
	std::shared_lock<std::shared_mutex> lock(cameraFrameMapMutex);
	auto cfEntry = cameraFrameMap.find(cameraID);
	if (cfEntry != cameraFrameMap.end()) {
		return cfEntry->second;
	} else {
		return {};
	}
}

std::optional<cam::CameraParams> getCameraIntrinsicParams(CameraID cameraID) {
	auto entry = cameraConfigMap.find(cameraID);
	if (entry != cameraConfigMap.end()) {
		auto cfg = entry->second;
		if (cfg.intrinsicParams) {
			return cfg.intrinsicParams;
		} else {
			return {};
		}
	} else {
		return {};
	}
}

std::optional<cv::Mat> getCameraExtrinsicParams(CameraID cameraID) {
	auto entry = cameraConfigMap.find(cameraID);
	if (entry != cameraConfigMap.end()) {
		auto cfg = entry->second;
		if (cfg.extrinsicParams) {
			return cfg.extrinsicParams;
		} else {
			return {};
		}
	} else {
		return {};
	}
}

double setCmdVel(double dtheta, double dx) {
	if (Globals::E_STOP && (dtheta != 0 || dx != 0)) {
		return 0;
	}

	wheelvel_t wheelVels = kinematics.robotVelToWheelVel(dx, dtheta);
	double lPWM = wheelVels.lVel / Constants::MAX_WHEEL_VEL;
	double rPWM = wheelVels.rVel / Constants::MAX_WHEEL_VEL;
	double maxAbsPWM = std::max(std::abs(lPWM), std::abs(rPWM));
	if (maxAbsPWM > 1) {
		lPWM /= maxAbsPWM;
		rPWM /= maxAbsPWM;
	}

	setCmdVelToIntegrate(wheelVels);
	setMotorPower(motorid_t::frontLeftWheel, lPWM);
	setMotorPower(motorid_t::frontRightWheel, lPWM);
	setMotorPower(motorid_t::frontRightWheel, rPWM);
	setMotorPower(motorid_t::rearRightWheel, rPWM);

	return maxAbsPWM > 1 ? maxAbsPWM : 1.0;
}

landmarks_t readLandmarks() {
	return AR::readLandmarks();
}

DataPoint<points_t> readLidarScan() {
	std::lock_guard<std::mutex> guard(lidarMutex);
	return lastLidar;
}

DataPoint<pose_t> readVisualOdomVel() {
	return DataPoint<pose_t>{};
}

DataPoint<double> readIMUHeading() {
	std::lock_guard<std::mutex> guard(headingMutex);
	return lastHeading;
}

DataPoint<pose_t> getTruePose() {
	std::lock_guard<std::mutex> guard(truePoseMutex);
	return lastTruePose;
}

URCLeg getLeg(int index) {
	return URCLeg{0, -1, {0., 0., 0.}};
}

void setMotorPower(motorid_t motor, double normalizedPWM) {
	std::string name = motorNameMap.at(motor);
	json msg = {{"type", "simMotorPowerRequest"}, {"motor", name}, {"power", normalizedPWM}};
	sendJSON(msg);
}

void setMotorPos(motorid_t motor, int32_t targetPos) {
	std::string name = motorNameMap.at(motor);
	json msg = {{"type", "simMotorPositionRequest"}, {"motor", name}, {"position", targetPos}};
	sendJSON(msg);
}

types::DataPoint<int32_t> getMotorPos(robot::types::motorid_t motor) {
	// TODO: return motor position
}

void setIndicator(indication_t signal) {
	if (signal == indication_t::arrivedAtDest) {
		log(LOG_INFO, "Robot arrived at destination!\n");
	}
}

} // namespace robot

DataPoint<gpscoords_t> gps::readGPSCoords() {
	std::lock_guard<std::mutex> guard(gpsMutex);
	return lastGPS;
}
