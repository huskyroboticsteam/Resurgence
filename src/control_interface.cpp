#include "control_interface.h"

#include "CAN/CAN.h"
#include "CAN/CANUtils.h"
#include "Constants.h"
#include "Globals.h"
#include "kinematics/DiffDriveKinematics.h"
#include "navtypes.h"
#include "utils/transform.h"
#include "world_interface/real_world_constants.h"
#include "world_interface/world_interface.h"

#include <atomic>
#include <chrono>
#include <loguru.hpp>
#include <mutex>
#include <thread>

using namespace navtypes;
using namespace robot::types;
using util::toTransform;

using control::DriveMode;
using Globals::swerveController;
using std::chrono::duration_cast;
using std::chrono::milliseconds;
using namespace std::chrono_literals;

namespace robot {
namespace {
kinematics::jointpos_t commandedWristPos{0, 0};
std::mutex wristPosMutex;

void setJointMotorPower(robot::types::jointid_t joint, double power);

std::unordered_map<types::jointid_t, double> jointPowerValues{};
std::mutex jointPowerValuesMutex;
void setJointPowerValue(types::jointid_t joint, double power);
double getJointPowerValue(types::jointid_t joint);
} // namespace

double setCmdVel(double dtheta, double dx) {
	if (isEmergencyStopped()) {
		return 0;
	}

	if (!Globals::swerveController.isOverridden()) {
		try {
			control::swerve_rots_t curr_wheel_rots = {
				robot::getMotorPos(motorid_t::frontLeftSwerve).getData(),
				robot::getMotorPos(motorid_t::frontRightSwerve).getData(),
				robot::getMotorPos(motorid_t::rearLeftSwerve).getData(),
				robot::getMotorPos(motorid_t::rearRightSwerve).getData()};
			if (!Globals::swerveController.checkWheelRotation(DriveMode::Normal,
															  curr_wheel_rots)) {
				return 0;
			}
		} catch (const bad_datapoint_access& e) {
			LOG_F(WARNING, "Invalid steer motor position(s)!");
			return 0;
		}
	}

	kinematics::wheelvel_t wheelVels = driveKinematics().robotVelToWheelVel(dx, dtheta);
	double lPWM = wheelVels.lVel / Constants::MAX_WHEEL_VEL;
	double rPWM = wheelVels.rVel / Constants::MAX_WHEEL_VEL;
	double maxAbsPWM = std::max(std::abs(lPWM), std::abs(rPWM));
	if (maxAbsPWM > 1) {
		lPWM /= maxAbsPWM;
		rPWM /= maxAbsPWM;
	}

	setMotorPower(motorid_t::frontLeftWheel, lPWM);
	setMotorPower(motorid_t::rearLeftWheel, lPWM);
	setMotorPower(motorid_t::frontRightWheel, rPWM);
	setMotorPower(motorid_t::rearRightWheel, rPWM);

	return maxAbsPWM > 1 ? maxAbsPWM : 1.0;
}

double setTankCmdVel(double left, double right) {
	if (isEmergencyStopped()) {
		return 0;
	}

	control::swerve_rots_t curr_wheel_rots;
	try {
		curr_wheel_rots = {robot::getMotorPos(motorid_t::frontLeftSwerve).getData(),
						   robot::getMotorPos(motorid_t::frontRightSwerve).getData(),
						   robot::getMotorPos(motorid_t::rearLeftSwerve).getData(),
						   robot::getMotorPos(motorid_t::rearRightSwerve).getData()};
	} catch (const bad_datapoint_access& e) {
		LOG_F(WARNING, "Invalid steer motor position(s)!");
		return 0;
	}
	if (!Globals::swerveController.checkWheelRotation(DriveMode::Normal, curr_wheel_rots))
		return 0;

	kinematics::wheelvel_t wheelVels = {left, right};
	double lPWM = wheelVels.lVel / Constants::MAX_WHEEL_VEL;
	double rPWM = wheelVels.rVel / Constants::MAX_WHEEL_VEL;
	double maxAbsPWM = std::max(std::abs(lPWM), std::abs(rPWM));
	if (maxAbsPWM > 1) {
		lPWM /= maxAbsPWM;
		rPWM /= maxAbsPWM;
	}

	setMotorPower(motorid_t::frontLeftWheel, lPWM);
	setMotorPower(motorid_t::rearLeftWheel, lPWM);
	setMotorPower(motorid_t::frontRightWheel, rPWM);
	setMotorPower(motorid_t::rearRightWheel, rPWM);

	return maxAbsPWM > 1 ? maxAbsPWM : 1.0;
}

void setJointPower(robot::types::jointid_t joint, double power) {
	// make sure power value is normalized
	if (std::abs(power) > 1) {
		power /= std::abs(power);
	}

	// store power value
	setJointPowerValue(joint, power);
	// set motor power
	setJointMotorPower(joint, power);
}

void setJointPos(robot::types::jointid_t joint, int32_t targetPos) {
	using robot::types::jointid_t;
	if (Constants::JOINT_MOTOR_MAP.find(joint) != Constants::JOINT_MOTOR_MAP.end()) {
		setMotorPos(Constants::JOINT_MOTOR_MAP.at(joint), targetPos);
	} else {
		LOG_F(WARNING, "setJointPos called for currently unsupported joint %s",
			  util::to_string(joint).c_str());
	}
}

types::DataPoint<int32_t> getJointPos(robot::types::jointid_t joint) {
	if (Constants::JOINT_MOTOR_MAP.find(joint) != Constants::JOINT_MOTOR_MAP.end()) {
		return getMotorPos(Constants::JOINT_MOTOR_MAP.at(joint));
	} else if (joint == jointid_t::ikForward || joint == jointid_t::ikUp) {
		DataPoint<navtypes::Vectord<Constants::arm::IK_MOTORS.size()>> armJointPositions =
			robot::getMotorPositionsRad(Constants::arm::IK_MOTORS);
		if (armJointPositions.isValid()) {
			Eigen::Vector2d eePos = Globals::planarArmController.kinematics().jointPosToEEPos(
				armJointPositions.getData());
			Eigen::Vector2i eePosInt = (1000 * eePos).array().round().cast<int>();
			return DataPoint<int32_t>(armJointPositions.getTime(),
									  joint == jointid_t::ikForward ? eePosInt.x()
																	: eePosInt.y());
		} else {
			return {};
		}
	} else if (joint == jointid_t::wristPitch || joint == jointid_t::wristRoll) {
		auto mdegToRad = [](int32_t mdeg) { return (mdeg / 1000.0) * (M_PI / 180.0); };
		auto lPos = getMotorPos(motorid_t::wristDiffLeft).transform(mdegToRad);
		auto rPos = getMotorPos(motorid_t::wristDiffRight).transform(mdegToRad);
		if (lPos.isValid() && rPos.isValid()) {
			kinematics::gearpos_t gearPos(lPos.getData(), rPos.getData());
			auto jointPos = Globals::wristKinematics.gearPosToJointPos(gearPos);
			float angle = joint == jointid_t::wristPitch ? jointPos.pitch : jointPos.roll;
			return DataPoint<int32_t>(lPos.getTime(),
									  static_cast<int32_t>(angle * (180.0 / M_PI) * 1000));
		} else {
			return {};
		}
	} else {
		// This should ideally never happen, but may if we haven't implemented a joint yet.
		LOG_F(WARNING, "getJointPos called for currently unsupported joint %s",
			  util::to_string(joint).c_str());
		return {};
	}
}

bool hasData(can::deviceid_t id) {
	auto start = std::chrono::steady_clock::now();
	constexpr int timeout_ms = 100;

	while (std::chrono::duration_cast<std::chrono::milliseconds>(
			   std::chrono::steady_clock::now() - start)
			   .count() < timeout_ms) {
		if (can::getDeviceTelemetry(id, can::telemtype_t::voltage)) {
			return true;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}
	return false;
}

void verifyAllMotorsConnected() {
	for (auto motorMap : robot::motorSerialIDMap) {
		auto motor_pair = std::make_pair(can::devicegroup_t::motor, motorMap.second);
		can::pullDeviceTelemetry(motor_pair, can::telemtype_t::voltage);
		if (!hasData(motor_pair)) {
			LOG_F(ERROR, "Motor not connected!\nID: %s",
				  std::to_string(motorMap.second).c_str());
			throw std::runtime_error("Not all motors connected!");
		}
	}
}

namespace {
void setJointPowerValue(types::jointid_t joint, double power) {
	// make sure power value is normalized
	if (std::abs(power) > 1) {
		power /= std::abs(power);
	}

	std::lock_guard<std::mutex> jointPwrValLock(jointPowerValuesMutex);
	jointPowerValues[joint] = power;
}

double getJointPowerValue(types::jointid_t joint) {
	std::lock_guard<std::mutex> jointPwrValLock(jointPowerValuesMutex);
	if (jointPowerValues.find(joint) == jointPowerValues.end()) {
		jointPowerValues.emplace(joint, 0);
	}
	return jointPowerValues.at(joint);
}

void setJointMotorPower(robot::types::jointid_t joint, double power) {
	using robot::types::jointid_t;
	if (Constants::JOINT_MOTOR_MAP.find(joint) != Constants::JOINT_MOTOR_MAP.end()) {
		bool isIKMotor = std::find(Constants::arm::IK_MOTOR_JOINTS.begin(),
								   Constants::arm::IK_MOTOR_JOINTS.end(),
								   joint) != Constants::arm::IK_MOTOR_JOINTS.end();
		if (!Globals::armIKEnabled || !isIKMotor) {
			setMotorPower(Constants::JOINT_MOTOR_MAP.at(joint), power);
		}
	} else if (joint == jointid_t::ikForward || joint == jointid_t::ikUp) {
		if (Globals::armIKEnabled) {
			auto ikMotorPos = robot::getMotorPositionsRad(Constants::arm::IK_MOTORS);
			if (ikMotorPos.isValid()) {
				if (joint == jointid_t::ikForward) {
					Globals::planarArmController.set_x_vel(dataclock::now(),
														   power * Constants::arm::MAX_EE_VEL,
														   ikMotorPos.getData());
				} else {
					Globals::planarArmController.set_y_vel(dataclock::now(),
														   power * Constants::arm::MAX_EE_VEL,
														   ikMotorPos.getData());
				}
			}
		}
	} else if (joint == jointid_t::wristPitch || joint == jointid_t::wristRoll) {
		setJointPowerValue(joint, power);
		kinematics::jointpos_t jointPwr(getJointPowerValue(jointid_t::wristPitch),
										getJointPowerValue(jointid_t::wristRoll));
		kinematics::gearpos_t gearPwr =
			Globals::wristKinematics.jointPowerToGearPower(jointPwr);
		setMotorPower(motorid_t::wristDiffLeft, gearPwr.left);
		setMotorPower(motorid_t::wristDiffRight, gearPwr.right);
	} else {
		LOG_F(WARNING, "setJointPower called for currently unsupported joint %s",
			  util::to_string(joint).c_str());
	}
}
} // namespace

} // namespace robot
