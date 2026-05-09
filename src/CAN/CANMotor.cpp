#include "CANMotor.h"

#include "CAN.h"
#include "CANUtils.h"

#include <chrono>
#include <cmath>
#include <thread>

#include <loguru.hpp>

extern "C" {
// new
#include <CANDevices.h>
#include <CANPacket.h>

#include <Packets/Motor.h>
#include <Packets/Universal.h>

// old
#include <HindsightCAN/CANCommon.h>
#include <HindsightCAN/CANMotorUnit.h>
#include <HindsightCAN/CANPacket.h>
#include <HindsightCAN/CANPower.h>
#include <HindsightCAN/CANScience.h>
}

using namespace std::chrono_literals;
using robot::types::DataPoint;
using robot::types::LimitSwitchData;

namespace can::motor {

// ===========
// UPDATED:
// ===========

CANBoard::CANBoard(robot::types::boardid_t board, bool hasPosSensor, CANDevice_t device,
				   double pos_pwm_scale, double neg_pwm_scale)
	: board_id(board), has_pos_sensor(hasPosSensor), device(device),
	  positive_scale(pos_pwm_scale), negative_scale(neg_pwm_scale) {
	std::lock_guard<std::mutex> lg(schedulerMutex);
	if (!pSched) {
		pSched.emplace("MotorVelSched");
	}
}

void CANBoard::ensureMotorMode(can::motor::motormode_t mode, can::motor::motorstate_t state) {
	ensureMotorMode(mode);
	if (!motor_state || motor_state.value() != state) {
		motor_state.emplace(state);
		can::motor::setMotorState(device, state);
	}
}

void CANBoard::ensureMotorMode(can::motor::motormode_t mode) {
	if (!motor_mode || motor_mode.value() != mode) {
		// update the motor mode
		motor_mode.emplace(mode);
		can::motor::setMotorMode(device, mode);
	}
}

void CANBoard::constructVelController() {
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
		[](const navtypes::Vectord<inputDim>&) {
			navtypes::Matrixd<outputDim, inputDim> res =
				navtypes::Matrixd<outputDim, inputDim>::Identity();
			return res;
		};

	velController.emplace(kinematicsFunct, jacobianFunct);
}

void CANBoard::setMotorPower(double power) {
	// scale the power
	double scale = power < 0 ? negative_scale : positive_scale;
	power *= scale;

	ensureMotorMode(can::motor::motormode_t::vel, can::motor::motorstate_t::control);
	can::motor::setMotorPower(device, power);

	if (power == 0.0 && static_cast<int>(board_id) < 4) { // hack
		ensureMotorMode(can::motor::motormode_t::vel, can::motor::motorstate_t::idle);
	}
}

void CANBoard::setMotorPos(int32_t targetPos) {
	ensureMotorMode(can::motor::motormode_t::pos);
	can::motor::setMotorPIDTarget(device, targetPos);
}

robot::types::DataPoint<int32_t> CANBoard::getMotorPos() const {
	return can::motor::getMotorPosition(device);
}

void CANBoard::setMotorVel(int32_t targetVel) {
	ensureMotorMode(can::motor::motormode_t::pos);
	if (!velController) {
		constructVelController();
	}

	navtypes::Vectord<1> velocityVector{targetVel};
	robot::types::datatime_t currTime = robot::types::dataclock::now();
	velController->setTarget(currTime, velocityVector);

	unscheduleVelocityEvent();

	velEventID = pSched->scheduleEvent(100ms, [this]() -> void {
		robot::types::datatime_t currTime = robot::types::dataclock::now();
		auto motorPos = can::motor::getMotorPosition(device);
		if (motorPos.isValid()) {
			const navtypes::Vectord<1> currPos(motorPos.getData());
			navtypes::Vectord<1> posCommand = velController->getCommand(currTime, currPos);
			setMotorPos(posCommand.coeff(0, 0));
		}
	});
}

void CANBoard::unscheduleVelocityEvent() {
	if (velEventID) {
		pSched->removeEvent(velEventID.value());
		velEventID.reset();
	}
}

can::uuid_t CANBoard::getMotorUUID() const {
	return device.deviceUUID;
}

robot::types::boardid_t CANBoard::getBoardID() const {
	return board_id;
}

void initEncoder(CANDevice_t device, bool invertEncoder, bool zeroEncoder,
				 int32_t pulsesPerJointRev,
				 std::optional<std::chrono::milliseconds> telemetryPeriod) {
	CANPacket_t p;
	// AssembleEncoderInitializePacket(&p, device, sensor_t::encoder, invertEncoder,
	// zeroEncoder);
	sendCANPacket(p);
	std::this_thread::sleep_for(1000us);
	if (telemetryPeriod) {
		scheduleTelemetryPull(device.deviceUUID, telemtype_t::angle, telemetryPeriod.value());
	}
}

void initPotentiometer(CANDevice_t device, int32_t posLo, int32_t posHi, uint16_t adcLo,
					   uint16_t adcHi,
					   std::optional<std::chrono::milliseconds> telemetryPeriod) {
	CANPacket_t p;
	/*
	AssemblePotHiSetPacket(&p, device, adcHi, posHi);
	p->id = ConstructCANID(PRIO_MOTOR_UNIT_POT_INIT, targetDeviceGroup, targetDeviceSerial);
	p->dlc = DLC_MOTOR_UNIT_POT_INIT;

	int idx = WritePacketIDOnly(p->data, ID_MOTOR_UNIT_POT_INIT_LO);
	PackShortIntoDataMSBFirst(p->data, adcLo, idx);
	idx += 2;
	PackIntIntoDataMSBFirst(p->data, mdegLo, idx);

	sendCANPacket(p);
	std::this_thread::sleep_for(1ms);
	AssemblePotLoSetPacket(&p, device, adcLo, posLo);
	sendCANPacket(p);
	*/
	if (telemetryPeriod) {
		scheduleTelemetryPull(device.deviceUUID, telemtype_t::angle, telemetryPeriod.value());
	}
}

void initMotor(CANDevice_t device) {
	setMotorMode(device, motormode_t::vel);
	std::this_thread::sleep_for(1000us);
}

void setMotorState(CANDevice_t device, motorstate_t state) {
	uint32_t axisState = static_cast<uint32_t>(state);
	CANPacket_t p = CANMotorPacket_BLDC_SetAxisState(Constants::JETSON_DEVICE, device, axisState);
	sendCANPacket(p);
}

void setMotorMode(CANDevice_t device, motormode_t mode) {
	// Map motormode_t to BLDC control/input modes
	uint8_t controlMode =
		(mode == motormode_t::pos) ? BLDC_POSITION_CONTROL : BLDC_VELOCITY_CONTROL;
	// uint8_t inputMode = BLDC_PASSTHROUGH_INPUT;
	uint8_t inputMode = BLDC_VEL_RAMP_INPUT;

	CANPacket_t p =
		CANMotorPacket_BLDC_SetInputMode(Constants::JETSON_DEVICE, device, controlMode, inputMode);
	sendCANPacket(p);
	std::this_thread::sleep_for(1000us);
}

void setMotorPower(CANDevice_t device, double power) {

	// Use BLDC velocity control: convert power [-1, 1] to velocity in rev/s
	// adjust as needed
	float velocity = static_cast<float>(power * 10.0); // 10 rev/s at full power
	float feedForwardTorque = 0.0f;

	CANPacket_t p = CANMotorPacket_BLDC_SetInputVelocity(Constants::JETSON_DEVICE, device, velocity,
														 feedForwardTorque);
	sendCANPacket(p);
}


void setMotorPIDTarget(CANDevice_t device, int32_t target) {
	// Convert millidegrees to revolutions
	float positionRev = static_cast<float>(target) / 360000.0f;
	float feedForwardVelocity = 0.0f;

	CANPacket_t p = CANMotorPacket_BLDC_SetInputPosition(Constants::JETSON_DEVICE, device, positionRev,
														 feedForwardVelocity);
	sendCANPacket(p);
}

DataPoint<int32_t> getMotorPosition(CANDevice_t device) {
	// Retrieve cached encoder position from the telemetry map
	return getDeviceTelemetry(device.deviceUUID, telemtype_t::angle);
}

void pullMotorPosition(CANDevice_t device) {
	// Request encoder estimates from the device
	uint8_t encoderID = 0; // Default encoder ID

	CANPacket_t p = CANMotorPacket_BLDC_GetEncoderEstimates(Constants::JETSON_DEVICE, device, encoderID);
	sendCANPacket(p);
}

void emergencyStopMotors() {
	// Broadcast e-stop to all domains
	CANDevice_t broadcast = {1, 1, 1, CAN_UUID_BROADCAST};
	CANPacket_t p = CANUniversalPacket_EStop(Constants::JETSON_DEVICE, broadcast);
	can::sendCANPacket(p);
	std::this_thread::sleep_for(1000us);
}

callbackid_t addLimitSwitchCallback(
	CANDevice_t device,
	const std::function<void(CANDevice_t device, DataPoint<LimitSwitchData> limitSwitchData)>&
		callback) {
	auto func = [device, callback](CANDeviceUUID_t, telemtype_t,
								   DataPoint<telemetry_t> telemData) {
		if (telemData) {
			LimitSwitchData data = telemData.getData();
			callback(device, DataPoint<LimitSwitchData>(telemData.getTime(), data));
		} else {
			callback(device, DataPoint<LimitSwitchData>());
		}
	};
	return addDeviceTelemetryCallback(device.deviceUUID, telemtype_t::limit_switch, func);
}

void removeLimitSwitchCallback(callbackid_t id) {
	removeDeviceTelemetryCallback(id);
}
} // namespace can::motor
