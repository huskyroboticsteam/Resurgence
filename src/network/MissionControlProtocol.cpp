#include "MissionControlProtocol.h"

#include "../Constants.h"
#include "../Globals.h"
#include "../autonomous/AutonomousTask.h"
#include "../control_interface.h"
#include "../utils/core.h"
#include "../utils/json.h"
#include "../world_interface/data.h"
#include "../world_interface/world_interface.h"
#include "MissionControlMessages.h"

#include <chrono>
#include <functional>
#include <loguru.hpp>
#include <unordered_set>

using namespace robot::types;
using namespace std::chrono_literals;

using val_t = nlohmann::json::value_t;
using control::DriveMode;
using Globals::swerveController;
using net::websocket::connhandler_t;
using net::websocket::msghandler_t;
using net::websocket::validator_t;
using robot::types::motorid_t;
using std::placeholders::_1;

namespace net::mc {
namespace {
const std::chrono::milliseconds HEARTBEAT_TIMEOUT_PERIOD = 3000ms;
} // namespace

/**
   Set power for all joints to zero.
 */
static void stopAllJoints();

/*///////////////// VALIDATORS/HANDLERS ////////////////////

  For each protocol message type, there is a pair of functions in this section: a validator and
  a handler. The validator will return a boolean depending on whether or not a message is valid
  for this type, and the handler will perform the required actions for dealing with a message
  of the type. NOTE: handlers expect valid messages, so call validator first
*/

static bool validateEmergencyStopRequest(const json& j) {
	return util::validateKey(j, "stop", val_t::boolean);
}

void MissionControlProtocol::handleEmergencyStopRequest(const json& j) {
	bool stop = j["stop"];
	if (stop) {
		this->stopAndShutdownPowerRepeat(true);
		robot::emergencyStop();
		LOG_F(ERROR, "Emergency stop!");
	} else if (!Globals::AUTONOMOUS) {
		// if we are leaving e-stop (and NOT in autonomous), restart the power repeater
		_power_repeat_task.start();
	}
	// TODO: reinit motors
	this->setArmIKEnabled(false);
}

static bool validateOperationModeRequest(const json& j) {
	return util::validateKey(j, "mode", val_t::string) &&
		   util::validateOneOf(j, "mode", {"teleoperation", "autonomous"});
}

void MissionControlProtocol::handleOperationModeRequest(const json& j) {
	std::string mode = j["mode"];
	Globals::AUTONOMOUS = (mode == "autonomous");
	if (Globals::AUTONOMOUS) {
		// if we have entered autonomous mode, we need to stop all the power repeater stuff.
		this->stopAndShutdownPowerRepeat(true);
	} else {
		_autonomous_task.kill();
		// if we have left autonomous mode, we need to start the power repeater again.
		_power_repeat_task.start();
	}
}

static bool validateDriveModeRequest(const json& j) {
	return util::validateKey(j, "mode", val_t::string) &&
		   util::validateOneOf(j, "mode", {"normal", "turn-in-place", "crab"}) &&
		   util::validateKey(j, "override", val_t::boolean);
}

void MissionControlProtocol::handleDriveModeRequest(const json& j) {
	std::string mode = j["mode"];
	control::swerve_rots_t target;
	if (mode == "normal") {
		target = swerveController.setDriveMode(DriveMode::Normal);
	} else if (mode == "turn-in-place") {
		target = swerveController.setDriveMode(DriveMode::TurnInPlace);
	} else if (mode == "crab") {
		target = swerveController.setDriveMode(DriveMode::Crab);
	} else {
		LOG_F(WARNING, "Invalid requested Drive Mode");
		return;
	}

	robot::setMotorPos(motorid_t::frontLeftSwerve, target.lfRot);
	robot::setMotorPos(motorid_t::rearLeftSwerve, target.lbRot);
	robot::setMotorPos(motorid_t::frontRightSwerve, target.rfRot);
	robot::setMotorPos(motorid_t::rearRightSwerve, target.rbRot);

	swerveController.setOverride(j["override"]);
}

static bool validateDriveRequest(const json& j) {
	if (swerveController.getDriveMode() != DriveMode::Normal) {
		LOG_F(WARNING, "Drive mode set to %s, not Normal",
			  util::to_string(swerveController.getDriveMode()).c_str());
	}
	return swerveController.getDriveMode() == DriveMode::Normal &&
		   util::hasKey(j, "straight") && util::validateRange(j, "straight", -1, 1) &&
		   util::hasKey(j, "steer") && util::validateRange(j, "steer", -1, 1);
}

static bool validateTankDriveRequest(const json& j) {
	if (swerveController.getDriveMode() != DriveMode::Normal) {
		LOG_F(WARNING, "Drive mode set to %s, not Normal",
			  util::to_string(swerveController.getDriveMode()).c_str());
	}
	return swerveController.getDriveMode() == DriveMode::Normal && util::hasKey(j, "left") &&
		   util::validateRange(j, "left", -1, 1) && util::hasKey(j, "right") &&
		   util::validateRange(j, "right", -1, 1);
}

static bool validateTurnInPlaceDriveRequest(const json& j) {
	if (swerveController.getDriveMode() != DriveMode::TurnInPlace) {
		LOG_F(WARNING, "Drive mode set to %s, not TurnInPlace",
			  util::to_string(swerveController.getDriveMode()).c_str());
	}
	return swerveController.getDriveMode() == DriveMode::TurnInPlace &&
		   util::hasKey(j, "steer") && util::validateRange(j, "steer", -1, 1);
}

static bool validateCrabDriveRequest(const json& j) {
	if (swerveController.getDriveMode() != DriveMode::Crab) {
		LOG_F(WARNING, "Drive mode set to %s, not Crab",
			  util::to_string(swerveController.getDriveMode()).c_str());
	}
	return swerveController.getDriveMode() == DriveMode::Crab && util::hasKey(j, "crab") &&
		   util::validateRange(j, "crab", -1, 1) && util::hasKey(j, "steer") &&
		   util::validateRange(j, "steer", -1, 1);
}

static bool validateArmIKEnable(const json& j) {
	return util::validateKey(j, "enabled", val_t::boolean);
}

void MissionControlProtocol::handleDriveRequest(const json& j) {
	// TODO: ignore this message if we are in autonomous mode.
	// fit straight and steer to unit circle; i.e. if |<straight, steer>| > 1, scale each
	// component such that <straight, steer> is a unit vector.
	double straight = j["straight"];
	double steer = -j["steer"].get<double>();
	double norm = std::hypot(straight, steer);
	double dx = Constants::MAX_WHEEL_VEL * (norm > 1 ? straight / norm : straight);
	double dtheta = Constants::MAX_DTHETA * (norm > 1 ? steer / norm : steer);
	LOG_F(1, "{straight=%.2f, steer=%.2f} -> setCmdVel(%.4f, %.4f)", straight, steer, dtheta,
		  dx);
	this->setRequestedCmdVel(dtheta, dx);
}

void MissionControlProtocol::handleTankDriveRequest(const json& j) {
	double left = j["left"];
	double right = j["right"];
	double leftVel = Constants::MAX_WHEEL_VEL * left;
	double rightVel = Constants::MAX_WHEEL_VEL * right;
	LOG_F(1, "{left=%.2f, right=%.2f} -> setTankCmdVel(%.4f, %.4f)", left, right, leftVel,
		  rightVel);
	this->setRequestedTankCmdVel(leftVel, rightVel);
}

void MissionControlProtocol::handleTurnInPlaceDriveRequest(const json& j) {
	double steer = -j["steer"].get<double>();
	double dtheta = Constants::MAX_DTHETA * steer;
	LOG_F(1, "{steer=%.2f} -> setTurnInPlaceCmdVel(%.4f)", steer, dtheta);
	this->setRequestedTurnInPlaceCmdVel(dtheta);
}

void MissionControlProtocol::handleCrabDriveRequest(const json& j) {
	double crab = -j["crab"].get<double>();
	double steer = -j["steer"].get<double>();
	double norm = std::hypot(crab, steer);
	double dy = Constants::MAX_WHEEL_VEL * (norm > 1 ? crab / norm : crab);
	double dtheta = Constants::MAX_DTHETA * (norm > 1 ? steer / norm : steer);
	LOG_F(1, "{crab=%.2f, steer=%.2f} -> setCrabCmdVel(%.4f, %.4f)", crab, steer, dtheta, dy);
	this->setRequestedCrabCmdVel(dtheta, dy);
}

void MissionControlProtocol::handleRequestArmIKEnabled(const json& j) {
	bool enabled = j["enabled"];
	if (enabled) {
		if (!Globals::armIKEnabled) {
			DataPoint<navtypes::Vectord<Constants::arm::IK_MOTORS.size()>> armJointPositions =
				robot::getMotorPositionsRad(Constants::arm::IK_MOTORS);

			bool success =
				armJointPositions.isValid() &&
				Globals::planarArmController.tryInitController(armJointPositions.getData());

			if (success) {
				this->setArmIKEnabled(true);
			} else {
				// unable to enable IK
				LOG_F(WARNING, "Unable to enable IK");
				this->setArmIKEnabled(false);
			}
		}
	} else {
		this->setArmIKEnabled(false);
	}
}

void MissionControlProtocol::setRequestedCmdVel(double dtheta, double dx) {
	_power_repeat_task.setCmdVel(dtheta, dx);
	robot::setCmdVel(dtheta, dx);
}

void MissionControlProtocol::setRequestedTankCmdVel(double left, double right) {
	_power_repeat_task.setTankCmdVel(left, right);
	robot::setTankCmdVel(left, right);
}

void MissionControlProtocol::setRequestedTurnInPlaceCmdVel(double dtheta) {
	_power_repeat_task.setTurnInPlaceCmdVel(dtheta);
}

void MissionControlProtocol::setRequestedCrabCmdVel(double dtheta, double dy) {
	_power_repeat_task.setCrabCmdVel(dtheta, dy);
}

static bool validateJoint(const json& j) {
	return util::validateKey(j, "joint", val_t::string) &&
		   std::any_of(all_jointid_t.begin(), all_jointid_t.end(), [&](const auto& joint) {
			   return j["joint"].get<std::string>() == util::to_string(joint);
		   });
}

static bool validateJointPowerRequest(const json& j) {
	return validateJoint(j) && util::validateRange(j, "power", -1, 1);
}

void MissionControlProtocol::handleJointPowerRequest(const json& j) {
	// TODO: ignore this message if we are in autonomous mode.
	using robot::types::jointid_t;
	using robot::types::name_to_jointid;
	std::string joint = j["joint"];
	double power = j["power"];
	auto it = name_to_jointid.find(util::freezeStr(joint));
	if (it != name_to_jointid.end()) {
		jointid_t joint_id = it->second;
		setRequestedJointPower(joint_id, power);
	}
}

static bool validateMotor(const json& j) {
	return util::validateKey(j, "motor", val_t::string) &&
		   std::any_of(all_motorid_t.begin(), all_motorid_t.end(), [&](const auto& motor) {
			   return j["motor"].get<std::string>() == util::to_string(motor);
		   });
}

static bool validateMotorPowerRequest(const json& j) {
	return validateMotor(j) && util::validateRange(j, "power", -1, 1);
}

void MissionControlProtocol::handleMotorPowerRequest(const json& j) {
	using robot::types::motorid_t;
	using robot::types::name_to_motorid;
	std::string motor = j["motor"];
	double power = j["power"];
	auto it = name_to_motorid.find(util::freezeStr(motor));
	if (it != name_to_motorid.end()) {
		motorid_t motor_id = it->second;
		setRequestedMotorPower(motor_id, power);
	}
}

static bool validateWaypointNavRequest(const json& j) {
	bool lat_is_unsigned = util::validateKey(j, "latitude", val_t::number_unsigned);
	bool lon_is_unsigned = util::validateKey(j, "longitude", val_t::number_unsigned);
	return (lat_is_unsigned || util::validateKey(j, "latitude", val_t::number_float)) &&
		   (lon_is_unsigned || util::validateKey(j, "longitude", val_t::number_float)) &&
		   util::validateKey(j, "isApproximate", val_t::boolean) &&
		   util::validateKey(j, "isGate", val_t::boolean);
}

void MissionControlProtocol::handleWaypointNavRequest(const json& j) {
	float latitude = j["latitude"];
	float longitude = j["longitude"];
	bool isApproximate = j["isApproximate"];
	bool isGate = j["isGate"];

	if (Globals::AUTONOMOUS && !isApproximate && !isGate) {
		navtypes::gpscoords_t coords = {latitude, longitude};
		auto target = robot::gpsToMeters(coords);
		if (target) {
			_autonomous_task.start(target.value());
		} else {
			LOG_F(WARNING, "No GPS converter initialized!");
		}
	}
}

static bool validateJointPositionRequest(const json& j) {
	return validateJoint(j) && util::validateKey(j, "position", val_t::number_integer);
}

static void handleJointPositionRequest([[maybe_unused]] const json& j) {
	// TODO: ignore this message if we are in autonomous mode.
	// std::string motor = j["joint"];
	// double position_deg = j["position"];
	// int32_t position_mdeg = std::round(position_deg * 1000);
	// TODO: actually implement joint position requests
	// setMotorPos(motor, position_mdeg);
}

static bool validateCameraStreamOpenRequest(const json& j) {
	return util::validateKey(j, "camera", val_t::string);
}

void MissionControlProtocol::handleCameraStreamOpenRequest(const json& j) {
	CameraID cam = j["camera"];
	std::unordered_set<CameraID> supported_cams = robot::getCameras();
	if (supported_cams.find(cam) != supported_cams.end()) {
		_camera_stream_task.openStream(cam, j["fps"]);
	}
}

static bool validateCameraStreamCloseRequest(const json& j) {
	return util::validateKey(j, "camera", val_t::string);
}

void MissionControlProtocol::handleCameraStreamCloseRequest(const json& j) {
	CameraID cam = j["camera"];
	_camera_stream_task.closeStream(cam);
}

void MissionControlProtocol::sendArmIKEnabledReport(bool enabled) {
	json msg = {{"type", ARM_IK_ENABLED_REP_TYPE}, {"enabled", enabled}};
	this->_server.sendJSON(Constants::MC_PROTOCOL_NAME, msg);
}

void MissionControlProtocol::handleConnection() {
	// Turn off inverse kinematics on connection
	this->setArmIKEnabled(false);

	// TODO: send the actual mounted peripheral, as specified by the command-line parameter
	json j = {{"type", MOUNTED_PERIPHERAL_REP_TYPE}};

	if (Globals::mountedPeripheral == mountedperipheral_t::none) {
		j["peripheral"] = json(nullptr);
	} else {
		j["peripheral"] = util::to_string(Globals::mountedPeripheral);
	}

	this->_server.sendJSON(Constants::MC_PROTOCOL_NAME, j);

	if (!Globals::AUTONOMOUS) {
		// start power repeat thread (if not already running)
		_power_repeat_task.start();
	}
}

void MissionControlProtocol::handleHeartbeatTimedOut() {
	LOG_F(ERROR, "Heartbeat timed out! Emergency stopping.");
	this->stopAndShutdownPowerRepeat(true);
	robot::emergencyStop();
}

void MissionControlProtocol::stopAndShutdownPowerRepeat(bool sendDisableIK) {
	if (_power_repeat_task.isRunning()) {
		_power_repeat_task.stop();
		// explicitly set all joints to zero
		stopAllJoints();
		// explicitly stop chassis
		robot::setCmdVel(0, 0);
	}
	// Turn off inverse kinematics so that IK state will be in sync with mission control
	this->setArmIKEnabled(false, sendDisableIK);
}

MissionControlProtocol::MissionControlProtocol(SingleClientWSServer& server)
	: WebSocketProtocol(Constants::MC_PROTOCOL_NAME), _server(server),
	  _camera_stream_task(server), _telem_report_task(server), _arm_ik_task(server),
	  _autonomous_task() {
	// TODO: Add support for tank drive requests
	// TODO: add support for science station requests (lazy susan, lazy susan lid, drill,
	// syringe)

	// emergency stop and operation mode handlers need the class for context since they must
	// be able to access the methods to start and stop the power repeater thread
	this->addMessageHandler(
		EMERGENCY_STOP_REQ_TYPE,
		std::bind(&MissionControlProtocol::handleEmergencyStopRequest, this, _1),
		validateEmergencyStopRequest);
	this->addMessageHandler(
		OPERATION_MODE_REQ_TYPE,
		std::bind(&MissionControlProtocol::handleOperationModeRequest, this, _1),
		validateOperationModeRequest);
	this->addMessageHandler(
		DRIVE_MODE_REQ_TYPE,
		std::bind(&MissionControlProtocol::handleDriveModeRequest, this, _1),
		validateDriveModeRequest);
	// drive and joint power handlers need the class for context since they must modify
	// _last_joint_power and _last_cmd_vel (for the repeater thread)
	this->addMessageHandler(DRIVE_REQ_TYPE,
							std::bind(&MissionControlProtocol::handleDriveRequest, this, _1),
							validateDriveRequest);
	this->addMessageHandler(
		DRIVE_TANK_REQ_TYPE,
		std::bind(&MissionControlProtocol::handleTankDriveRequest, this, _1),
		validateTankDriveRequest);
	this->addMessageHandler(
		DRIVE_TURN_IN_PLACE_REQ_TYPE,
		std::bind(&MissionControlProtocol::handleTurnInPlaceDriveRequest, this, _1),
		validateTurnInPlaceDriveRequest);
	this->addMessageHandler(
		DRIVE_CRAB_REQ_TYPE,
		std::bind(&MissionControlProtocol::handleCrabDriveRequest, this, _1),
		validateCrabDriveRequest);
	this->addMessageHandler(
		ARM_IK_ENABLED_TYPE,
		std::bind(&MissionControlProtocol::handleRequestArmIKEnabled, this, _1),
		validateArmIKEnable);
	this->addMessageHandler(
		JOINT_POWER_REQ_TYPE,
		std::bind(&MissionControlProtocol::handleJointPowerRequest, this, _1),
		validateJointPowerRequest);
	this->addMessageHandler(
		MOTOR_POWER_REQ_TYPE,
		std::bind(&MissionControlProtocol::handleMotorPowerRequest, this, _1),
		validateMotorPowerRequest);
	this->addMessageHandler(JOINT_POSITION_REQ_TYPE, handleJointPositionRequest,
							validateJointPositionRequest);
	this->addMessageHandler(
		CAMERA_STREAM_OPEN_REQ_TYPE,
		std::bind(&MissionControlProtocol::handleCameraStreamOpenRequest, this, _1),
		validateCameraStreamOpenRequest);
	this->addMessageHandler(
		CAMERA_STREAM_CLOSE_REQ_TYPE,
		std::bind(&MissionControlProtocol::handleCameraStreamCloseRequest, this, _1),
		validateCameraStreamCloseRequest);
	this->addMessageHandler(
		WAYPOINT_NAV_REQ_TYPE,
		std::bind(&MissionControlProtocol::handleWaypointNavRequest, this, _1),
		validateWaypointNavRequest);
	this->addConnectionHandler(std::bind(&MissionControlProtocol::handleConnection, this));
	this->addDisconnectionHandler(
		std::bind(&MissionControlProtocol::stopAndShutdownPowerRepeat, this, false));

	this->setHeartbeatTimedOutHandler(
		HEARTBEAT_TIMEOUT_PERIOD,
		std::bind(&MissionControlProtocol::handleHeartbeatTimedOut, this));

	_telem_report_task.start();
	_camera_stream_task.start();
}

MissionControlProtocol::~MissionControlProtocol() {
	this->stopAndShutdownPowerRepeat(true);
}

void MissionControlProtocol::setArmIKEnabled(bool enabled, bool sendReport) {
	Globals::armIKEnabled = enabled;
	if (enabled) {
		_arm_ik_task.start();
	} else {
		_arm_ik_task.stop();
	}

	if (sendReport) {
		sendArmIKEnabledReport(enabled);
	}
}

void MissionControlProtocol::setRequestedJointPower(jointid_t joint, double power) {
	_power_repeat_task.setJointPower(joint, power);
	robot::setJointPower(joint, power);
}

void MissionControlProtocol::setRequestedMotorPower(motorid_t motor, double power) {
	_power_repeat_task.setMotorPower(motor, power);
	robot::setMotorPower(motor, power);
}

///// UTILITY FUNCTIONS //////

static void stopAllJoints() {
	for (jointid_t current : robot::types::all_jointid_t) {
		robot::setJointPower(current, 0.0);
	}
}

} // namespace net::mc
