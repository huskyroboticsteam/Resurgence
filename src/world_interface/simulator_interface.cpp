#include "../Constants.h"
#include "../base64/base64_img.h"
#include "../camera/Camera.h"
#include "../camera/CameraConfig.h"
#include "../kinematics/DiffDriveKinematics.h"
#include "../navtypes.h"
#include "../network/websocket/WebSocketProtocol.h"
#include "../utils/core.h"
#include "../utils/transform.h"
#include "motor/sim_motor.h"
#include "world_interface.h"

#include <atomic>
#include <condition_variable>
#include <loguru.hpp>
#include <mutex>
#include <shared_mutex>
#include <string>
#include <unordered_map>

#include <nlohmann/json.hpp>
#include <opencv2/core.hpp>

using nlohmann::json;
using namespace navtypes;
using namespace robot::types;
using namespace cam;

namespace {
const std::string PROTOCOL_PATH("/simulator");
const std::map<motorid_t, std::string> motorNameMap = {
	{motorid_t::frontLeftWheel, "frontLeftWheel"},
	{motorid_t::frontLeftSwerve, "frontLeftSwerve"},
	{motorid_t::frontRightWheel, "frontRightWheel"},
	{motorid_t::frontRightSwerve, "frontRightSwerve"},
	{motorid_t::rearLeftWheel, "rearLeftWheel"},
	{motorid_t::rearLeftSwerve, "rearLeftSwerve"},
	{motorid_t::rearRightWheel, "rearRightWheel"},
	{motorid_t::rearRightSwerve, "rearRightSwerve"},
	{motorid_t::armBase, "armBase"},
	{motorid_t::shoulder, "shoulder"},
	{motorid_t::elbow, "elbow"},
	{motorid_t::forearm, "forearm"},
	{motorid_t::wristDiffLeft, "wristDiffLeft"},
	{motorid_t::wristDiffRight, "wristDiffRight"},
	{motorid_t::hand, "hand"},
	{motorid_t::activeSuspension, "activeSuspension"},
	{motorid_t::drillActuator, "drillActuator"}};

std::optional<std::reference_wrapper<net::websocket::SingleClientWSServer>> wsServer;

DataPoint<Eigen::Quaterniond> lastOrientation;
std::mutex orientationMutex;

DataPoint<gpscoords_t> lastGPS;
std::mutex gpsMutex;

DataPoint<pose_t> lastTruePose;
std::mutex truePoseMutex;

std::map<std::string, DataPoint<int32_t>> motorPosMap;
std::shared_mutex motorPosMapMutex;

bool is_emergency_stopped = false;

// A mapping of (motor_id, shared pointer to object of the motor)
std::unordered_map<robot::types::motorid_t, std::shared_ptr<robot::base_motor>> motor_ptrs;

using lscallback_t =
	std::function<void(robot::types::DataPoint<LimitSwitchData> limitSwitchData)>;
std::map<std::string, std::map<robot::callbackid_t, lscallback_t>> limitSwitchCallbackMap;
robot::callbackid_t nextCallbackID = 0;
std::map<robot::callbackid_t, motorid_t> lsCallbackToMotorMap;
std::mutex limitSwitchCallbackMapMutex; // protects both maps and nextCallbackID

// stores the last camera frame for each camera
std::unordered_map<CameraID, DataPoint<CameraFrame>> cameraFrameMap;
// stores the index of the last camera frame for each camera
// cameraFrameMap is not guaranteed to have valid data points,
// but this one is guaranteed to have indices, if there are any
std::unordered_map<CameraID, uint32_t> cameraLastFrameIdxMap;
std::shared_mutex cameraFrameMapMutex; // protects both of the above maps
// not modified after startup, no need to synchronize
std::unordered_map<CameraID, cam::CameraConfig> cameraConfigMap;

std::condition_variable connectionCV;
std::mutex connectionMutex;
bool simConnected = false;

void sendJSON(const json& obj) {
	wsServer->get().sendJSON(PROTOCOL_PATH, obj);
}

static void openCamera(CameraID cam, std::optional<std::vector<double>> list1d = std::nullopt,
					   uint8_t fps = 20, uint16_t width = 640, uint16_t height = 480) {
	if (list1d) {
		json msg = {{"type", "simCameraStreamOpenRequest"},
					{"camera", cam},
					{"fps", fps},
					{"width", width},
					{"height", height},
					{"intrinsics", list1d.value()}};
		sendJSON(msg);
	} else {
		json msg = {{"type", "simCameraStreamOpenRequest"},
					{"camera", cam},
					{"fps", fps},
					{"width", width},
					{"height", height},
					{"intrinsics", nullptr}};
		sendJSON(msg);
	}
}

void initCameras() {
	auto cfg = cam::readConfigFromFile(Constants::MAST_CAMERA_CONFIG_PATH);
	cameraConfigMap[Constants::MAST_CAMERA_ID] = cfg;
	openCamera(Constants::HAND_CAMERA_ID, cfg.intrinsicParams->getIntrinsicList());
	openCamera(Constants::FOREARM_CAMERA_ID, cfg.intrinsicParams->getIntrinsicList());
	openCamera(Constants::MAST_CAMERA_ID, cfg.intrinsicParams->getIntrinsicList());
}

void initMotors() {
	// initializes map of motor ids and shared ptrs of their objects
	for (const auto& x : motorNameMap) {
		std::shared_ptr<robot::base_motor> ptr =
			std::make_shared<robot::sim_motor>(x.first, true, x.second, PROTOCOL_PATH);
		motor_ptrs.insert({x.first, ptr});
	}
}

void handleGPS(json msg) {
	gpscoords_t coords{msg["latitude"], msg["longitude"]};
	std::lock_guard<std::mutex> guard(gpsMutex);
	lastGPS = {coords};
}

void handleIMU(json msg) {
	double qx = msg["x"];
	double qy = msg["y"];
	double qz = msg["z"];
	double qw = msg["w"];

	Eigen::Quaterniond q(qw, qx, qy, qz);
	q.normalize();
	std::lock_guard<std::mutex> guard(orientationMutex);
	lastOrientation = q;
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
	std::string motorName = msg["motor"];
	auto posJson = msg["position"];
	DataPoint<int32_t> posData;
	if (!posJson.is_null()) {
		int32_t pos = posJson;
		posData = {pos};
	}

	std::unique_lock lock(motorPosMapMutex);
	motorPosMap.insert_or_assign(motorName, posData);
}

void handleLimitSwitch(json msg) {
	std::string motorName = msg["motor"];
	uint8_t data;
	std::string limit = msg["limit"];
	if (limit == "maximum") {
		data = 1 << LIMIT_SWITCH_LIM_MAX_IDX;
	} else if (limit == "minimum") {
		data = 1 << LIMIT_SWITCH_LIM_MIN_IDX;
	}
	DataPoint<LimitSwitchData> lsData(data);

	std::lock_guard lock(limitSwitchCallbackMapMutex);
	for (const auto& entry : limitSwitchCallbackMap.at(motorName)) {
		entry.second(lsData);
	}
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
	LOG_F(INFO, "Simulator connected!");
	{
		std::lock_guard<std::mutex> lock(connectionMutex);
		simConnected = true;
	}
	connectionCV.notify_one();
}

void clientDisconnected() {
	LOG_F(ERROR, "ERROR: Simulator disconnected! World Interface disconnected!");
}

void initSimServer(net::websocket::SingleClientWSServer& ws) {
	wsServer = ws;
	auto protocol = std::make_unique<net::websocket::WebSocketProtocol>(PROTOCOL_PATH);
	protocol->addMessageHandler("simImuOrientationReport", handleIMU);
	protocol->addMessageHandler("simGpsPositionReport", handleGPS);
	protocol->addMessageHandler("simCameraStreamReport", handleCamFrame);
	protocol->addMessageHandler("simMotorStatusReport", handleMotorStatus);
	protocol->addMessageHandler("simLimitSwitchAlert", handleLimitSwitch);
	protocol->addMessageHandler("simRoverTruePoseReport", handleTruePose);
	protocol->addConnectionHandler(clientConnected);
	protocol->addDisconnectionHandler(clientDisconnected);

	wsServer->get().addProtocol(std::move(protocol));

	{
		// wait for simulator to connect
		LOG_F(INFO, "Waiting for simulator connection...");
		std::unique_lock<std::mutex> lock(connectionMutex);
		connectionCV.wait(lock, [] { return simConnected; });
	}
}

} // namespace

namespace robot {

namespace {
kinematics::DiffDriveKinematics drive_kinematics(Constants::EFF_WHEEL_BASE);
} // namespace

const kinematics::DiffDriveKinematics& driveKinematics() {
	return drive_kinematics;
}

extern const WorldInterface WORLD_INTERFACE = WorldInterface::sim3d;

void world_interface_init(
	std::optional<std::reference_wrapper<net::websocket::SingleClientWSServer>> wsServer,
	bool initOnlyMotors) {
	initSimServer(wsServer.value());
	initCameras();
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
	for (const auto& motor : motorNameMap) {
		setMotorPower(motor.first, 0.0);
	}
	is_emergency_stopped = true;
}

bool isEmergencyStopped() {
	return is_emergency_stopped;
}

std::unordered_set<CameraID> getCameras() {
	std::shared_lock<std::shared_mutex> lock(cameraFrameMapMutex);
	return util::keySet(cameraLastFrameIdxMap);
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

landmarks_t readLandmarks() {
	return {};
}

DataPoint<Eigen::Quaterniond> readIMU() {
	std::lock_guard<std::mutex> guard(orientationMutex);
	return lastOrientation;
}

DataPoint<pose_t> getTruePose() {
	std::lock_guard<std::mutex> guard(truePoseMutex);
	return lastTruePose;
}

void setMotorPower(motorid_t motor, double normalizedPWM) {
	if (isEmergencyStopped()) {
		LOG_F(ERROR, "E-Stopped! Ignoring call for: setMotorPower");
		return;
	}
	std::shared_ptr<robot::base_motor> motor_ptr = getMotor(motor);
	motor_ptr->setMotorPower(normalizedPWM);
}

void setMotorPos(motorid_t motor, int32_t targetPos) {
	if (isEmergencyStopped()) {
		LOG_F(ERROR, "E-Stopped! Ignoring call for: setMotorPos");
		return;
	}
	std::shared_ptr<robot::base_motor> motor_ptr = getMotor(motor);
	motor_ptr->setMotorPos(targetPos);
}

DataPoint<int32_t> getMotorPos(motorid_t motor) {
	auto itr = motorNameMap.find(motor);
	if (itr != motorNameMap.end()) {
		std::string motorName = itr->second;
		std::shared_lock lock(motorPosMapMutex);
		auto entry = motorPosMap.find(motorName);
		if (entry != motorPosMap.end()) {
			return entry->second;
		} else {
			return {};
		}
	} else {
		return {};
	}
}

void setMotorVel(robot::types::motorid_t motor, int32_t targetVel) {
	if (isEmergencyStopped()) {
		LOG_F(ERROR, "E-Stopped! Ignoring call for: setMotorVel");
		return;
	}
	std::shared_ptr<robot::base_motor> motor_ptr = getMotor(motor);
	motor_ptr->setMotorVel(targetVel);
}

callbackid_t addLimitSwitchCallback(
	robot::types::motorid_t motor,
	const std::function<void(robot::types::motorid_t motor,
							 robot::types::DataPoint<LimitSwitchData> limitSwitchData)>&
		callback) {
	auto itr = motorNameMap.find(motor);
	if (itr != motorNameMap.end()) {
		std::string motorName = itr->second;
		auto func = std::bind(callback, motor, std::placeholders::_1);

		std::lock_guard lock(limitSwitchCallbackMapMutex);
		limitSwitchCallbackMap.insert({motorName, {}});
		callbackid_t nextID = nextCallbackID++;
		limitSwitchCallbackMap.at(motorName).insert({nextID, func});
		lsCallbackToMotorMap.insert({nextID, motor});
		return nextID;
	} else {
		return nextCallbackID++;
	}
}

void removeLimitSwitchCallback(callbackid_t id) {
	std::lock_guard lock(limitSwitchCallbackMapMutex);
	std::string motorName = motorNameMap.at(lsCallbackToMotorMap.at(id));
	limitSwitchCallbackMap.at(motorName).erase(id);
	if (limitSwitchCallbackMap.at(motorName).empty()) {
		limitSwitchCallbackMap.erase(motorName);
	}
}

void setIndicator(indication_t signal) {
	if (signal == indication_t::arrivedAtDest) {
		LOG_F(INFO, "Robot arrived at destination!");
	}
}

} // namespace robot

DataPoint<gpscoords_t> gps::readGPSCoords() {
	std::lock_guard<std::mutex> guard(gpsMutex);
	return lastGPS;
}
