#include "CANMotor.h"
#include "CAN.h"
#include "CANUtils.h"

#include <chrono>
#include <cmath>
#include <thread>

extern "C" {
// new
#include <CANPacket.h>
#include <Packets/Motor.h>
#include <Packets/Universal.h>
#include <CANDevices.h>

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

// Jetson device as sender
static const CANDevice_t JETSON_DEVICE = {0, 0, 0, CAN_UUID_JETSON};

void initEncoder() {
	// TO DO
}

void initMotor(CANDevice_t device) {
	setMotorMode(device, motormode_t::pwm);
	std::this_thread::sleep_for(1000us);
}

void setMotorMode(CANDevice_t device, motormode_t mode) {
	// Map motormode_t to BLDC control/input modes
	uint8_t controlMode = (mode == motormode_t::pid) ? BLDC_POSITION_CONTROL : BLDC_VELOCITY_CONTROL;
	uint8_t inputMode = BLDC_PASSTHROUGH_INPUT;

	CANPacket_t p = CANMotorPacket_BLDC_SetInputMode(JETSON_DEVICE, device, controlMode, inputMode);
	sendCANPacket(p);
	std::this_thread::sleep_for(1000us);
}

void setMotorPower(CANDevice_t device, double power) {
	// Clamp power to [-1.0, 1.0]
	power = std::min(std::max(power, -1.0), 1.0);

	// Use BLDC velocity control: convert power [-1, 1] to velocity in rev/s
	// adjust as needed
	float velocity = static_cast<float>(power * 10.0);  // 10 rev/s at full power
	float feedForwardTorque = 0.0f;

	CANPacket_t p = CANMotorPacket_BLDC_SetInputVelocity(JETSON_DEVICE, device, velocity, feedForwardTorque);
	sendCANPacket(p);
}

void setMotorPower(CANDevice_t device, int16_t power) {
	// Convert int16_t power to double [-1.0, 1.0]
	double powerDouble = static_cast<double>(power) / std::numeric_limits<int16_t>::max();
	setMotorPower(device, powerDouble);
}

void setMotorPIDTarget(CANDevice_t device, int32_t target) {
	// Convert millidegrees to revolutions
	float positionRev = static_cast<float>(target) / 360000.0f;
	float feedForwardVelocity = 0.0f;

	CANPacket_t p = CANMotorPacket_BLDC_SetInputPosition(JETSON_DEVICE, device, positionRev, feedForwardVelocity);
	sendCANPacket(p);
}

DataPoint<int32_t> getMotorPosition(CANDevice_t device) {
	// Retrieve cached encoder position from the telemetry map
	return getDeviceTelemetry(device.deviceUUID, telemtype_t::angle);
}

void pullMotorPosition(CANDevice_t device) {
	// Request encoder estimates from the device
	uint8_t encoderID = 0;  // Default encoder ID

	CANPacket_t p = CANMotorPacket_BLDC_GetEncoderEstimates(JETSON_DEVICE, device, encoderID);
	sendCANPacket(p);
}

void emergencyStopMotors() {
	// Broadcast e-stop to all domains
	CANDevice_t broadcast = {1, 1, 1, CAN_UUID_BROADCAST};
	CANPacket_t p = CANUniversalPacket_EStop(JETSON_DEVICE, broadcast);
	can::sendCANPacket(p);
	std::this_thread::sleep_for(1000us);
}

callbackid_t addLimitSwitchCallback(
	CANDevice_t device,
	const std::function<void(CANDevice_t device,
							 DataPoint<LimitSwitchData> limitSwitchData)>& callback) {
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

// ===========
// DEPRECATED:
// ===========

/*
void initEncoder(devicegroup_t group, deviceserial_t serial, bool invertEncoder,
				 bool zeroEncoder, int32_t pulsesPerJointRev,
				 std::optional<std::chrono::milliseconds> telemetryPeriod) {
	auto groupCode = static_cast<uint8_t>(group);
	CANPacket_t p;
	AssembleEncoderInitializePacket(&p, groupCode, serial, sensor_t::encoder, invertEncoder,
									zeroEncoder);
	sendCANPacket(p);
	std::this_thread::sleep_for(1000us);
	AssembleEncoderPPJRSetPacket(&p, groupCode, serial, pulsesPerJointRev);
	sendCANPacket(p);
	std::this_thread::sleep_for(1000us);
	if (telemetryPeriod) {
		scheduleTelemetryPull(std::make_pair(group, serial), telemtype_t::angle,
							  telemetryPeriod.value());
	}
}

void initPotentiometer(devicegroup_t group, deviceserial_t serial, int32_t posLo,
					   int32_t posHi, uint16_t adcLo, uint16_t adcHi,
					   std::optional<std::chrono::milliseconds> telemetryPeriod) {
	CANPacket packet;
	auto groupCode = static_cast<uint8_t>(group);
	AssemblePotHiSetPacket(&packet, groupCode, serial, adcHi, posHi);
	sendCANPacket(packet);
	std::this_thread::sleep_for(1ms);
	AssemblePotLoSetPacket(&packet, groupCode, serial, adcLo, posLo);
	sendCANPacket(packet);
	if (telemetryPeriod) {
		scheduleTelemetryPull(std::make_pair(group, serial), telemtype_t::angle,
							  telemetryPeriod.value());
	}
}

void initMotor(devicegroup_t group, deviceserial_t serial) {
	setMotorMode(group, serial, motormode_t::pwm);
	std::this_thread::sleep_for(1000us);
}

void setLimitSwitchLimits(devicegroup_t group, deviceserial_t serial, int32_t lo, int32_t hi) {
	CANPacket p;
	auto groupCode = static_cast<uint8_t>(group);

	// 0 is low limit, 1 is high limit.
	AssembleLimSwEncoderBoundPacket(&p, groupCode, serial, 0, lo);
	sendCANPacket(p);
	std::this_thread::sleep_for(1ms);
	AssembleLimSwEncoderBoundPacket(&p, groupCode, serial, 1, hi);
	sendCANPacket(p);
}

void setMotorPIDConstants(devicegroup_t group, deviceserial_t serial, int32_t kP, int32_t kI,
						  int32_t kD) {
	CANPacket p;
	auto boardGroupCode = static_cast<uint8_t>(group);
	AssemblePSetPacket(&p, boardGroupCode, serial, kP);
	sendCANPacket(p);
	std::this_thread::sleep_for(1ms);
	AssembleISetPacket(&p, boardGroupCode, serial, kI);
	sendCANPacket(p);
	std::this_thread::sleep_for(1ms);
	AssembleDSetPacket(&p, boardGroupCode, serial, kD);
	sendCANPacket(p);
	std::this_thread::sleep_for(1ms);
}

void setMotorMode(devicegroup_t group, deviceserial_t serial, motormode_t mode) {
	CANPacket p;
	AssembleModeSetPacket(&p, static_cast<uint8_t>(group), serial, static_cast<uint8_t>(mode));
	sendCANPacket(p);
	std::this_thread::sleep_for(1000us);
}

void setMotorPower(devicegroup_t group, deviceserial_t serial, double power) {
	power = std::min(std::max(power, -1.0), 1.0);
	int powerInt = std::round(power * std::numeric_limits<int16_t>::max());
	int16_t dutyCycle = static_cast<int16_t>(powerInt);
	setMotorPower(group, serial, dutyCycle);
}

void setMotorPower(devicegroup_t group, deviceserial_t serial, int16_t power) {
	CANPacket p;
	AssemblePWMDirSetPacket(&p, static_cast<uint8_t>(group), serial, power);
	sendCANPacket(p);
}

void setMotorPIDTarget(devicegroup_t group, deviceserial_t serial, int32_t target) {
	CANPacket p;
	AssemblePIDTargetSetPacket(&p, static_cast<uint8_t>(group), serial, target);
	sendCANPacket(p);
}

DataPoint<int32_t> getMotorPosition(devicegroup_t group, deviceserial_t serial) {
	return getDeviceTelemetry(std::make_pair(group, serial), telemtype_t::angle);
}

void pullMotorPosition(devicegroup_t group, deviceserial_t serial) {
	pullDeviceTelemetry(std::make_pair(group, serial), telemtype_t::angle);
}

callbackid_t addLimitSwitchCallback(
	devicegroup_t group, deviceserial_t serial,
	const std::function<void(devicegroup_t group, deviceserial_t serial,
							 DataPoint<LimitSwitchData> limitSwitchData)>& callback) {
	auto id = std::make_pair(group, serial);
	// wrap callback in lambda to change signature
	auto func = [callback](deviceid_t deviceID, telemtype_t,
						   DataPoint<telemetry_t> telemData) {
		if (telemData) {
			LimitSwitchData data = telemData.getData();
			callback(deviceID.first, deviceID.second, {telemData.getTime(), data});
		} else {
			callback(deviceID.first, deviceID.second, {});
		}
	};
	return addDeviceTelemetryCallback(id, telemtype_t::limit_switch, func);
}
*/
} // namespace can::motor
