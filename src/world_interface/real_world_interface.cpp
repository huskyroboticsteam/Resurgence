#include "../CAN/CANBoard.h"
#include "../Constants.h"
#include "../ardupilot/ArduPilotInterface.h"
#include "../camera/Camera.h"
#include "../control/JacobianVelController.h"
#include "../gps/usb_gps/read_usb_gps.h"
#include "../navtypes.h"
#include "../utils/core.h"
#include "../utils/scheduler.h"
#include "real_world_constants.h"
#include "world_interface.h"

#include <future>
#include <mutex>
#include <unordered_map>
#include <vector>

#include <loguru.hpp>
#include <opencv2/calib3d.hpp>

using namespace navtypes;
using namespace std::chrono_literals;

namespace robot {

namespace {

// A mapping of (board_id, shared pointer to board)
std::unordered_map<robot::types::boardid_t, std::shared_ptr<can::CANBoard>> board_ptrs;

kinematics::DiffDriveKinematics drive_kinematics(Constants::EFF_WHEEL_BASE);
bool is_emergency_stopped = false;

std::shared_ptr<can::CANBoard> getBoard_(robot::types::boardid_t board) {
	auto itr = board_ptrs.find(board);

	if (itr == board_ptrs.end()) {
		// board id not in map
		LOG_F(ERROR, "Unknown board 0x%x", static_cast<uint8_t>(board));
		return nullptr;
	} else {
		// return board object pointer
		return itr->second;
	}
}

// map that associates camera id to the camera object
std::unordered_map<CameraID, std::weak_ptr<cam::Camera>> cameraMap;

void initBoards() {
	// Initialize boards using CANDevice_t from boardDeviceMap
	for (const auto& [board, device] : boardDeviceMap) {
		if (auto it = boardDeviceMap.find(board); it != boardDeviceMap.end()) {
			// create ptr and insert in map
			std::shared_ptr<can::CANBoard> ptr = std::make_shared<can::CANBoard>(board, it->second);
			board_ptrs.insert({board, ptr});
		} else {
			LOG_F(ERROR, "Couldn't find UUID mapping for board 0x%x", static_cast<uint8_t>(board));
			return;
		}
	}
}

std::shared_ptr<cam::Camera> openCamera_(CameraID camID) {
	auto it = cameraMap.find(camID);
	if (it != cameraMap.end()) {
		auto cam = it->second.lock();
		if (cam) {
			return cam;
		}
	}
	try {
		auto cam = std::make_shared<cam::Camera>();
		bool success = cam->open(camID);
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

const kinematics::DiffDriveKinematics& driveKinematics() {
	return drive_kinematics;
}

void world_interface_init(
	std::optional<std::reference_wrapper<net::websocket::SingleClientWSServer>> wsServer,
	bool initOnlyMotors) {
	if (!initOnlyMotors) {
		if (wsServer.has_value()) {
			ardupilot::initArduPilotProtocol(wsServer.value());
		}
	}
	can::initCAN();
	initBoards();
}

std::shared_ptr<types::CameraHandle> openCamera(CameraID cameraID) {
	std::shared_ptr<cam::Camera> cam = openCamera_(cameraID);
	return cam ? std::make_shared<types::CameraHandle>(cam) : nullptr;
}

void emergencyStop() {
	can::emergencyStop();
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

robot::types::DataPoint<robot::types::CameraFrame> readCamera(CameraID cameraID) {
	auto itr = cameraMap.find(cameraID);
	if (itr != cameraMap.end()) {
		auto cam = itr->second.lock();
		if (!cam) {
			LOG_F(WARNING, "Cam %s not available", cameraID.c_str());
			return robot::types::DataPoint<robot::types::CameraFrame>{};
		}
		cv::Mat mat;
		uint32_t frameNum;
		robot::types::datatime_t time;
		bool success = cam->next(mat, frameNum, time);
		if (success) {
			return robot::types::DataPoint<robot::types::CameraFrame>{time, {mat, frameNum}};
		} else {
			return robot::types::DataPoint<robot::types::CameraFrame>{};
		}
	} else {
		LOG_F(WARNING, "Invalid camera id: %s", cameraID.c_str());
		return robot::types::DataPoint<robot::types::CameraFrame>{};
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

robot::types::DataPoint<pose_t> getTruePose() {
	return {};
}

robot::types::landmarks_t readLandmarks() {
	return {};
}

template <typename T>
int getIndex(const std::vector<T>& vec, const T& val) {
	auto itr = std::find(vec.begin(), vec.end(), val);
	return itr == vec.end() ? -1 : itr - vec.begin();
}

void setMotorPower(robot::types::boardid_t board, double power) {
	std::shared_ptr<can::CANBoard> board_ptr = getBoard_(board);
	if (board_ptr) {
		board_ptr->setMotorPower(power);
	}
}

void setMotorPos(robot::types::boardid_t board, int32_t targetPos) {
	std::shared_ptr<can::CANBoard> board_ptr = getBoard_(board);
	if (board_ptr) {
		// TODO: Implement
		// board_ptr->setMotorPos(targetPos);
	}
}

robot::types::DataPoint<int32_t> getMotorPos(robot::types::boardid_t board) {
	std::shared_ptr<can::CANBoard> board_ptr = getBoard_(board);
	if (board_ptr) {
		return board_ptr->getPosition();
	}
	return {};
}

void setMotorVel(robot::types::boardid_t board, int8_t targetVel) {
	std::shared_ptr<can::CANBoard> board_ptr = getBoard_(board);
	if (board_ptr) {
		board_ptr->setMotorVel(targetVel);
	}
}

callbackid_t addLimitSwitchCallback(
	robot::types::boardid_t board,
	const std::function<void(robot::types::boardid_t board,
								robot::types::DataPoint<robot::types::LimitSwitchData> limitSwitchData)>&
		callback) {
	// CAN26: Use CANDevice_t for limit switch callbacks
	// CANDevice_t device = boardDeviceMap.at(board);
	// auto func = [=](CANDevice_t, robot::types::DataPoint<robot::types::LimitSwitchData> data) { callback(board, data); };
	// auto id = can::motor::addLimitSwitchCallback(device, func);
	// auto nextID = nextCallbackID++;
	// callbackIDMap.insert({nextID, id});
	// return nextID;
	return 0;
}

void removeLimitSwitchCallback(callbackid_t id) {
	// return can::motor::removeLimitSwitchCallback(callbackIDMap.at(id));
}

void handleMotorEncoderEstimate(robot::types::boardid_t board, int32_t positionMdeg) {
	std::shared_ptr<can::CANBoard> board_ptr = getBoard_(board);
	if (board_ptr) {
		board_ptr->storePosition(robot::types::DataPoint<int32_t>(positionMdeg));
	}
}

void setStepperRevs(robot::types::boardid_t board, float revs) {
	std::shared_ptr<can::CANBoard> board_ptr = getBoard_(board);
	if (board_ptr) {
		board_ptr->setStepperRevs(revs);
	}
}

void setActuator(int8_t out) {
	std::shared_ptr<can::CANBoard> board_ptr = getBoard_(robot::types::boardid_t::hand);
	if (board_ptr) {
		board_ptr->setActuator(out);
	}
}

void setPeripheralPWM(uint8_t peripheralID, float dutyCycle) {
	std::shared_ptr<can::CANBoard> board_ptr = getBoard_(robot::types::boardid_t::hand);
	if (board_ptr) {
		board_ptr->setPWMDutyCycle(peripheralID, dutyCycle);
	}
}

void setLED(uint8_t color) {
	can::led_t led = static_cast<can::led_t>(color);
	can::setLED(led);
}

} // namespace robot