#include "../Constants.h"
#include "../Globals.h"
#include "../kinematics/DiffDriveKinematics.h"
#include "../navtypes.h"
#include "../utils/transform.h"
#include "world_interface.h"

#include <atomic>
#include <chrono>
#include <loguru.hpp>
#include <mutex>

using namespace navtypes;
using namespace robot::types;
using util::toTransform;

using std::chrono::duration_cast;
using std::chrono::milliseconds;
using namespace std::chrono_literals;

namespace robot {
namespace {
DataPoint<transform_t> lastOdom;
wheelvel_t commandedWheelVel{0, 0};
jointpos_t commandedWristPos{0, 0};
std::mutex wristPosMutex;

void setCmdVelToIntegrate(const wheelvel_t& wheelVels) {
	auto odom = robot::readOdom();
	lastOdom = odom;
	commandedWheelVel = wheelVels;
}

void setJointMotorPower(robot::types::jointid_t joint, double power);

std::unordered_map<types::jointid_t, double> jointPowerValues{};
std::mutex jointPowerValuesMutex;
void setJointPowerValue(types::jointid_t joint, double power);
double getJointPowerValue(types::jointid_t joint);
} // namespace

DataPoint<transform_t> readOdom() {
	if (!lastOdom) {
		return toTransform({0, 0, 0});
	} else {
		datatime_t now = dataclock::now();
		double elapsed =
			duration_cast<milliseconds>(now - lastOdom.getTime()).count() / 1000.0;
		transform_t update =
			toTransform(driveKinematics().getLocalPoseUpdate(commandedWheelVel, elapsed));
		transform_t odom = lastOdom.getData() * update;
		lastOdom = {now, odom};
		return lastOdom;
	}
}

double setCmdVel(double dtheta, double dx) {
	if (Globals::E_STOP && (dtheta != 0 || dx != 0)) {
		return 0;
	}

	wheelvel_t wheelVels = driveKinematics().robotVelToWheelVel(dx, dtheta);
	double lPWM = wheelVels.lVel / Constants::MAX_WHEEL_VEL;
	double rPWM = wheelVels.rVel / Constants::MAX_WHEEL_VEL;
	double maxAbsPWM = std::max(std::abs(lPWM), std::abs(rPWM));
	if (maxAbsPWM > 1) {
		lPWM /= maxAbsPWM;
		rPWM /= maxAbsPWM;
	}

	setCmdVelToIntegrate(wheelVels);
	setMotorPower(motorid_t::frontLeftWheel, lPWM);
	setMotorPower(motorid_t::rearLeftWheel, lPWM);
	setMotorPower(motorid_t::frontRightWheel, rPWM);
	setMotorPower(motorid_t::rearRightWheel, rPWM);

	return maxAbsPWM > 1 ? maxAbsPWM : 1.0;
}

std::pair<double, double> getCmdVel() {
	double l = commandedWheelVel.lVel;
	double r = commandedWheelVel.rVel;
	pose_t robotVel = driveKinematics().wheelVelToRobotVel(l, r);
	return {robotVel(2), robotVel(0)};
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
	}
	// FIXME: need to do some extra control (probably implementing our own PID control) for the
	// differential position, since the potentiometers are giving us joint position instead of
	// motor position.
	/*
	else if (joint == jointid_t::differentialPitch || joint == jointid_t::differentialRoll) {
		std::lock_guard<std::mutex> lk(wristPosMutex);
		if (joint == jointid_t::differentialPitch){
			commandedWristPos.pitch = targetPos;
		} else {
			commandedWristPos.roll = targetPos;
		}
		gearpos_t gearPos = wristKinematics().jointPosToGearPos(commandedWristPos);
		setMotorPos(motorid_t::differentialLeft, gearPos.left);
		setMotorPos(motorid_t::differentialRight, gearPos.right);
	}
	*/
	else {
		// FIXME: this should ideally never happen, but we don't have support for all joints
		// yet because we don't know anything about the drill arm (and need to do extra work
		// for the differential)
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
			Eigen::Vector2d eePos =
				Globals::planarArmKinematics.jointPosToEEPos(armJointPositions.getData());
			Eigen::Vector2i eePosInt = (1000 * eePos).array().round().cast<int>();
			return DataPoint<int32_t>(armJointPositions.getTime(),
									  joint == jointid_t::ikForward ? eePosInt.x()
																	: eePosInt.y());
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
			if (joint == jointid_t::ikForward) {
				Globals::planarArmController.set_x_vel(dataclock::now(),
													   power * Constants::arm::MAX_EE_VEL);
			} else {
				Globals::planarArmController.set_y_vel(dataclock::now(),
													   power * Constants::arm::MAX_EE_VEL);
			}
		}
	} else {
		LOG_F(WARNING, "setJointPower called for currently unsupported joint %s",
			  util::to_string(joint).c_str());
	}
}
} // namespace

} // namespace robot
