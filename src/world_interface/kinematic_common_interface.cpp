#include "../Constants.h"
#include "../Globals.h"
#include "../Util.h"
#include "../kinematics/DiffDriveKinematics.h"
#include "../control/JacobianVelController.h"
#include "../CAN/CANUtils.h"
#include "../log.h"
#include "../navtypes.h"
#include "world_interface.h"
#include "real_world_constants.h"

#include <chrono>
#include <mutex>
#include <atomic>

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
const std::unordered_map<robot::types::jointid_t, robot::types::motorid_t> jointMotorMap{
	{robot::types::jointid_t::armBase, robot::types::motorid_t::armBase},
	{robot::types::jointid_t::shoulder, robot::types::motorid_t::shoulder},
	{robot::types::jointid_t::elbow, robot::types::motorid_t::elbow},
	{robot::types::jointid_t::forearm, robot::types::motorid_t::forearm},
	{robot::types::jointid_t::wrist, robot::types::motorid_t::wrist},
	{robot::types::jointid_t::hand, robot::types::motorid_t::hand}};

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

void setMotorVel(robot::types::motorid_t motor, int32_t targetVel) {
	// thread will run when called for the first time (std::thread in global) / velocity thread
	// if no thread running, spin up a thread
	// velocity task (Evan wrote this a while ago)
	// at fixed update rate: iterate over motors, 

	// main pitfalls: setMotorPower may be controlled by velocity task or other tasks and function is in another place
		// how do we know if it's being controlled by a task?

	// make motor an object
		// getMotor() in world interface
		// shared ptr for polymorphism
		// different implementation depending on world interface (init method in world interfaces - sim motor, can motor)

	// get motor position and set its dimensions
	types::DataPoint<int32_t> motorPos = getMotorPos(motor);
	if (!motorPos.isValid()) {
		return;
	}
	constexpr int32_t inputDim = 1;
	constexpr int32_t outputDim = 1;

	// create vectors for motor position and velocity
	navtypes::Vectord<inputDim> posVector {motorPos.getData()};
	navtypes::Vectord<inputDim> velocityVector {targetVel};

	// create kinematics function (input and output will both be the current motor position)
	const std::function<navtypes::Vectord<outputDim>(const navtypes::Vectord<inputDim>&)>& kinematicsFunct = 
		[](const navtypes::Vectord<inputDim>& inputVec)->navtypes::Vectord<outputDim> { 
		// returns a copy of the input vector
		navtypes::Vectord<outputDim> res {inputVec(0)};
		return res; }
	;

	// create jacobian function (value will be 1 since it's the derivative of the kinematics function)
	const std::function<navtypes::Matrixd<outputDim, inputDim>(const navtypes::Vectord<inputDim>&)>& jacobianFunct = 
		[](const navtypes::Vectord<inputDim>& inputVec)->navtypes::Matrixd<outputDim, inputDim> { 
			navtypes::Matrixd<outputDim, inputDim> res = navtypes::Matrixd<outputDim, inputDim>::Ones();
			//res << 1;
		}
	;

	// get the current time
	types::datatime_t currTime = types::dataclock::now();

	// set the target velocity
	JacobianVelController<outputDim, inputDim> velController(kinematicsFunct, jacobianFunct);
	velController.setTarget(currTime, velocityVector);
	assert(("Target velocity should be set in setMotorVel", velController.hasTarget()));

	
	//return velController.getCommand(currTime, posVector);
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
	if (jointMotorMap.find(joint) != jointMotorMap.end()) {
		setMotorPos(jointMotorMap.at(joint), targetPos);
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
		log(LOG_WARN, "setJointPower called for currently unsupported joint %s\n",
			util::to_string(joint).c_str());
	}
}
types::DataPoint<int32_t> getJointPos(robot::types::jointid_t joint) {
	if(jointMotorMap.find(joint) != jointMotorMap.end()){
		return getMotorPos(jointMotorMap.at(joint));
	}
	// FIXME: need to do some extra work for differential - we will have to figure out which
	// motor boards the potentiometers are plugged into and query those for "motor position"
	else {
		// FIXME: this should ideally never happen, but we don't have support for all joints
		// yet because we don't know anything about the drill arm (and need to do extra work
		// for the differential)
		log(LOG_WARN, "getJointPower called for currently unsupported joint %s\n",
			util::to_string(joint).c_str());
		return {};
	}
}

namespace {
void setJointPowerValue(types::jointid_t joint, double power){
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

	if (jointMotorMap.find(joint) != jointMotorMap.end()) {
		setMotorPower(jointMotorMap.at(joint), power);
	} else {
		log(LOG_WARN, "setJointPower called for currently unsupported joint %s\n",
			util::to_string(joint).c_str());
	}
}
} // namespace

} // namespace robot
