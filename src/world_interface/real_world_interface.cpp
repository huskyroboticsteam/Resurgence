#include "../CAN/CAN.h"
#include "../CAN/CANMotor.h"
#include "../CAN/CANUtils.h"
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
#include <iostream>
#include <loguru.hpp>
#include <mutex>
#include <unordered_map>
#include <vector>

#include <opencv2/calib3d.hpp>

using nlohmann::json;
using namespace navtypes;
using namespace robot::types;
using can::motor::motormode_t;
using namespace std::chrono_literals;

namespace robot {

extern const WorldInterface WORLD_INTERFACE = WorldInterface::real;

namespace {

class CANBoard {
public:
	// CAN constructor
	CANBoard(robot::types::boardid_t motor, bool hasPosSensor,
			 can::uuid_t uuid, can::domainmask_t domains,
			 double pos_pwm_scale, double neg_pwm_scale)
		: motor_id(motor),
		  has_pos_sensor(hasPosSensor),
		  board_uuid(uuid),
		  board_domains(domains),
		  positive_scale(pos_pwm_scale),
		  negative_scale(neg_pwm_scale) {
		// create scheduler if needed
		std::lock_guard<std::mutex> lg(schedulerMutex);
		if (!pSched) {
			pSched.emplace("MotorVelSched");
		}
	}

	/* LEGACY constructor (group/serial addressing) - commented out for CAN2026 migration
	CANBoard(robot::types::boardid_t motor, bool hasPosSensor,
			 can::deviceserial_t serial, can::devicegroup_t group,
			 double pos_pwm_scale, double neg_pwm_scale)
		: motor_id(motor),
		  has_pos_sensor(hasPosSensor),
		  serial_id(serial),
		  device_group(group),
		  positive_scale(pos_pwm_scale),
		  negative_scale(neg_pwm_scale) {
		std::lock_guard<std::mutex> lg(schedulerMutex);
		if (!pSched) {
			pSched.emplace("MotorVelSched");
		}
	}
	*/

	void setMotorPower(double power) {
		ensureMotorMode(can::motor::motormode_t::pwm);

		// scale the power
		double scale = power < 0 ? negative_scale : positive_scale;
		power *= scale;
		can::motor::setMotorPower(board_uuid, board_domains, power);
		// LEGACY: can::motor::setMotorPower(device_group, serial_id, power);
	}

	void setMotorPos(int32_t targetPos) {
		ensureMotorMode(can::motor::motormode_t::pid);
		can::motor::setMotorPIDTarget(board_uuid, board_domains, targetPos);
		// LEGACY: can::motor::setMotorPIDTarget(device_group, serial_id, targetPos);
	}

	types::DataPoint<int32_t> getMotorPos() const {
		return can::motor::getMotorPosition(board_uuid, board_domains);
		// LEGACY: return can::motor::getMotorPosition(device_group, serial_id);
	}

	void setMotorVel(int32_t targetVel) {
		ensureMotorMode(can::motor::motormode_t::pid);
		if (!velController) {
			constructVelController();
		}
		// set velocity target
		navtypes::Vectord<1> velocityVector{targetVel};
		types::datatime_t currTime = types::dataclock::now();
		velController->setTarget(currTime, velocityVector);

		// check to see if the event exists. if yes, unschedule it
		unscheduleVelocityEvent();

		// schedule position event
		velEventID = pSched->scheduleEvent(100ms, [this]() -> void {
			types::datatime_t currTime = types::dataclock::now();
			auto motorPos = getMotorPos();
			if (motorPos.isValid()) {
				const navtypes::Vectord<1> currPos{getMotorPos().getData()};
				navtypes::Vectord<1> posCommand = velController->getCommand(currTime, currPos);
				setMotorPos(posCommand.coeff(0, 0));
			}
		});
	}
	
	void unscheduleVelocityEvent() {
		if (velEventID) {
			pSched->removeEvent(velEventID.value());
			velEventID.reset();
		}
	}

	robot::types::boardid_t getMotorID() const {
		return motor_id;
	}

	can::uuid_t getMotorUUID() const {
		return board_uuid;
	}

private:
	robot::types::boardid_t motor_id;
	bool has_pos_sensor;
	can::uuid_t board_uuid;
	can::domainmask_t board_domains;
	/* LEGACY:
	can::deviceserial_t serial_id;
	can::devicegroup_t device_group;
	*/
	std::optional<can::motor::motormode_t> motor_mode;
	double positive_scale;
	double negative_scale;
	std::optional<util::PeriodicScheduler<std::chrono::steady_clock>::eventid_t> velEventID;
	std::optional<JacobianVelController<1, 1>> velController;
	
	inline static std::optional<util::PeriodicScheduler<std::chrono::steady_clock>> pSched;
	inline static std::mutex schedulerMutex;

	void ensureMotorMode(can::motor::motormode_t mode) {
		if (!motor_mode || motor_mode.value() != mode) {
			// update the motor mode
			motor_mode.emplace(mode);
			can::motor::setMotorMode(board_uuid, board_domains, mode);
			// LEGACY: can::motor::setMotorMode(device_group, serial_id, mode);
		}
	}	

	void constructVelController() {
		// define dimensions
		constexpr int32_t inputDim = 1;
		constexpr int32_t outputDim = 1;

		// create kinematics function (input and output will both be the current motor
		// position)
		const std::function<navtypes::Vectord<outputDim>(const navtypes::Vectord<inputDim>&)>&
			kinematicsFunct = [](const navtypes::Vectord<inputDim>& inputVec) {
				// returns a copy of the input vector
				return inputVec;
			};

		// create jacobian function (value will be 1 since it's the derivative of the
		// kinematics function)
		const std::function<navtypes::Matrixd<outputDim, inputDim>(
			const navtypes::Vectord<inputDim>&)>& jacobianFunct =
			[](const navtypes::Vectord<inputDim>& inputVec) {
				navtypes::Matrixd<outputDim, inputDim> res =
					navtypes::Matrixd<outputDim, inputDim>::Identity();
				return res;
			};

		velController.emplace(kinematicsFunct, jacobianFunct);
	}
};

// A mapping of (motor_id, shared pointer to object of the motor)
std::unordered_map<robot::types::boardid_t, std::shared_ptr<CANBoard>> motor_ptrs;

kinematics::DiffDriveKinematics drive_kinematics(Constants::EFF_WHEEL_BASE);
bool is_emergency_stopped = false;

void addMotorMapping(boardid_t motor, bool hasPosSensor) {
	double posScale = 0;
	double negScale = 0;

	// get scales for motor
	try {
		posScale = positive_pwm_scales.at(motor);
		negScale = negative_pwm_scales.at(motor);
	} catch (const std::out_of_range& err) {
		LOG_F(ERROR, "Couldn't find PWM scales for motor 0x%x", static_cast<uint8_t>(motor));
	}

	// get UUID and domains from boardUUIDMap
	can::deviceinfo_t deviceInfo;
	try {
		// deviceInfo = boardUUIDMap.at(motor);
	} catch (const std::out_of_range& err) {
		LOG_F(ERROR, "Couldn't find UUID mapping for motor 0x%x", static_cast<uint8_t>(motor));
		return;
	}

	// create ptr and insert in map
	std::shared_ptr<CANBoard> ptr =
		std::make_shared<CANBoard>(motor, hasPosSensor, deviceInfo.uuid, deviceInfo.domains,
								   posScale, negScale);
	/* 
	LEGACY:
	std::shared_ptr<CANBoard> ptr =
		std::make_shared<CANBoard>(motor, hasPosSensor, boardSerialIDMap.at(motor),
								   boardGroupMap.at(motor), posScale, negScale);
	*/
	motor_ptrs.insert({motor, ptr});
}

std::shared_ptr<CANBoard> getMotor_(robot::types::boardid_t motor) {
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
	// CAN2026: Initialize motors using UUID-based addressing
	/* for (const auto& it : boardUUIDMap) {
		boardid_t motor = it.first;
		can::deviceinfo_t info = it.second;
		can::motor::initMotor(info.uuid, info.domains);
		bool hasPosSensor = robot::potMotors.find(motor) != robot::potMotors.end() ||
							robot::encMotors.find(motor) != robot::encMotors.end();
		addMotorMapping(motor, hasPosSensor);
	}

	for (const auto& pot_motor_pair : robot::potMotors) {
		boardid_t motor_id = pot_motor_pair.first;
		potparams_t pot_params = pot_motor_pair.second;

		// CAN2026: Use UUID-based addressing
		can::deviceinfo_t info = boardUUIDMap.at(motor_id);
		can::motor::initPotentiometer(info.uuid, info.domains, pot_params.mdeg_lo, pot_params.mdeg_hi,
									  pot_params.adc_lo, pot_params.adc_hi, TELEM_PERIOD); */
		/* LEGACY:
		can::devicegroup_t group = boardGroupMap.at(motor_id);
		can::deviceserial_t serial = boardSerialIDMap.at(motor_id);
		can::motor::initPotentiometer(group, serial, pot_params.mdeg_lo, pot_params.mdeg_hi,
									  pot_params.adc_lo, pot_params.adc_hi, TELEM_PERIOD);
	}
	*/

	for (const auto& enc_motor_pair : robot::encMotors) {
		boardid_t motor_id = enc_motor_pair.first;
		encparams_t enc_params = enc_motor_pair.second;

		// CAN2026: Use UUID-based addressing
/* 		can::deviceinfo_t info = boardUUIDMap.at(motor_id);
		can::motor::initEncoder(info.uuid, info.domains, enc_params.isInverted, true, enc_params.ppjr,
								TELEM_PERIOD);
		can::motor::setLimitSwitchLimits(info.uuid, info.domains, enc_params.limitSwitchLow,
										 enc_params.limitSwitchHigh); */
		/* LEGACY:
		can::devicegroup_t group = boardGroupMap.at(motor_id);
		can::deviceserial_t serial = boardSerialIDMap.at(motor_id);
		can::motor::initEncoder(group, serial, enc_params.isInverted, true, enc_params.ppjr,
								TELEM_PERIOD);
		can::motor::setLimitSwitchLimits(group, serial, enc_params.limitSwitchLow,
										 enc_params.limitSwitchHigh);
		*/
	}

	for (const auto& pair : robot::motorPIDMap) {
		boardid_t motor = pair.first;
		pidcoef_t pid = motorPIDMap.at(motor);
		// CAN2026: Use UUID-based addressing
/* 		can::deviceinfo_t info = boardUUIDMap.at(motor);
		can::motor::setMotorPIDConstants(info.uuid, info.domains, pid.kP, pid.kI, pid.kD); */
		/* LEGACY:
		can::devicegroup_t group = boardGroupMap.at(motor);
		can::deviceserial_t serial = boardSerialIDMap.at(motor);
		can::motor::setMotorPIDConstants(group, serial, pid.kP, pid.kI, pid.kD);
		*/
	}
}

std::shared_ptr<cam::Camera> openCamera_(CameraID camID) {
	auto it = cameraMap.find(camID);
	if (it != cameraMap.end()) {
		auto cam = it->second.lock();
		if (cam) { return cam; }
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

void setMotorPower(robot::types::boardid_t motor, double power) {
/* 	std::shared_ptr<CANBoard> motor_ptr = getMotor_(motor);
	if (motor_ptr) {
		motor_ptr->setMotorPower(power);
	} */
}

void setMotorPos(robot::types::boardid_t motor, int32_t targetPos) {
/* 	std::shared_ptr<CANBoard> motor_ptr = getMotor_(motor);
	if (motor_ptr) {
		motor_ptr->setMotorPos(targetPos);
	} */
}

types::DataPoint<int32_t> getMotorPos(robot::types::boardid_t motor) {
/* 	std::shared_ptr<CANBoard> motor_ptr = getMotor_(motor);
	if (motor_ptr) {
		return motor_ptr->getMotorPos();
	}
	return {}; */
}

void setMotorVel(robot::types::boardid_t motor, int32_t targetVel) {
/* 	std::shared_ptr<CANBoard> motor_ptr = getMotor_(motor);
	motor_ptr->setMotorVel(targetVel);
	if (motor_ptr) {
		motor_ptr->setMotorVel(targetVel);
	} */
}

callbackid_t addLimitSwitchCallback(
	robot::types::boardid_t motor,
	const std::function<void(robot::types::boardid_t motor,
							 robot::types::DataPoint<LimitSwitchData> limitSwitchData)>&
		callback) {
	// CAN2026: Use UUID-based addressing for limit switch callbacks
 	can::deviceinfo_t info = boardUUIDMap.at(motor);
	auto func = [=](can::uuid_t uuid, can::domainmask_t domains,
					DataPoint<LimitSwitchData> data) { callback(motor, data); };
	auto id = can::motor::addLimitSwitchCallback(info.uuid, info.domains, func);
	/* LEGACY:
	auto func = [=](can::devicegroup_t group, can::deviceserial_t serial,
					DataPoint<LimitSwitchData> data) { callback(motor, data); };
	auto id = can::motor::addLimitSwitchCallback(boardGroupMap.at(motor),
												 boardSerialIDMap.at(motor), func);
	*/
	auto nextID = nextCallbackID++;
	callbackIDMap.insert({nextID, id});
	return nextID;
}

void removeLimitSwitchCallback(callbackid_t id) {
	return can::motor::removeLimitSwitchCallback(callbackIDMap.at(id));
}

} // namespace robot
