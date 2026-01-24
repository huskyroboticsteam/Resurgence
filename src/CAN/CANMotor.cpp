#include "CANMotor.h"

#include "CAN.h"
#include "CANUtils.h"

#include <chrono>
#include <cmath>
#include <thread>

extern "C" {
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

void initEncoder(uuid_t uuid, domainmask_t domains, bool invertEncoder,
				 bool zeroEncoder, int32_t pulsesPerJointRev,
				 std::optional<std::chrono::milliseconds> telemetryPeriod);

void initMotor(uuid_t uuid, domainmask_t domains) {
	setMotorMode(uuid, domains, motormode_t::pwm);
	std::this_thread::sleep_for(1000us);
}

void setLimitSwitchLimits(uuid_t uuid, domainmask_t domains, int32_t lo, int32_t hi) {
	/*
	TODO: Uncomment once HindsightCAN is updated for CAN2026

	CANPacket p;
	auto groupCode = static_cast<uint8_t>(group);

	// 0 is low limit, 1 is high limit.
	AssembleLimSwEncoderBoundPacket(&p, groupCode, serial, 0, lo);
	sendCANPacket(p);
	std::this_thread::sleep_for(1ms);
	AssembleLimSwEncoderBoundPacket(&p, groupCode, serial, 1, hi);
	sendCANPacket(p);
	*/
}

void setMotorPIDConstants(uuid_t uuid, domainmask_t domains, int32_t kP, int32_t kI,
						  int32_t kD) {
	/*
	TODO: Uncomment once HindsightCAN is updated for CAN2026

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
	*/
}

void setMotorMode(uuid_t uuid, domainmask_t domains, motormode_t mode) {
	/*
	TODO: Uncomment once HindsightCAN is updated for CAN2026

	CANPacket p;
	AssembleModeSetPacket(&p, static_cast<uint8_t>(group), serial, static_cast<uint8_t>(mode));
	sendCANPacket(p);
	std::this_thread::sleep_for(1000us);
	*/
}

void setMotorPower(uuid_t uuid, domainmask_t domains, double power) {
	/*
	TODO: Uncomment once HindsightCAN is updated for CAN2026

	power = std::min(std::max(power, -1.0), 1.0);
	int powerInt = std::round(power * std::numeric_limits<int16_t>::max());
	int16_t dutyCycle = static_cast<int16_t>(powerInt);
	setMotorPower(uuid, domains, dutyCycle);
	*/
}

void setMotorPower(uuid_t uuid, domainmask_t domains, int16_t power) {
	/*
	TODO: Uncomment once HindsightCAN is updated for CAN2026

	CANPacket p;
	AssemblePWMDirSetPacket(&p, static_cast<uint8_t>(group), serial, power);
	sendCANPacket(p);
	*/
}

void setMotorPIDTarget(uuid_t uuid, domainmask_t domains, int32_t target) {
	/* 	
	TODO: Uncomment once HindsightCAN is updated for CAN2026

	CANPacket p;
	AssemblePIDTargetSetPacket(&p, static_cast<uint8_t>(group), serial, target);
	sendCANPacket(p); 
	*/
}

DataPoint<int32_t> getMotorPosition(uuid_t uuid, domainmask_t domains) {
	/* 
	TODO: Uncomment once HindsightCAN is updated for CAN2026

	return getDeviceTelemetry(std::make_pair(group, serial), telemtype_t::angle); 
	*/
}

void pullMotorPosition(uuid_t uuid, domainmask_t domains) {
	/* 	
	TODO: Uncomment once HindsightCAN is updated for CAN2026

	pullDeviceTelemetry(std::make_pair(group, serial), telemtype_t::angle); 
	*/
}

callbackid_t addLimitSwitchCallback(
	uuid_t uuid, domainmask_t domains,
	const std::function<void(uuid_t uuid, domainmask_t domains,
							 DataPoint<LimitSwitchData> limitSwitchData)>& callback) {
	/* 	
	TODO: Uncomment once HindsightCAN is updated for CAN2026
	
	devicegroup_t group = (domains & static_cast<uint8_t>(domain_t::motor))
						  ? devicegroup_t::motor
						  : devicegroup_t::science;
	deviceserial_t serial = uuid;

	auto id = std::make_pair(group, serial);
	auto func = [callback, uuid, domains](deviceid_t deviceID, telemtype_t,
										  DataPoint<telemetry_t> telemData) {
		if (telemData) {
			LimitSwitchData data = telemData.getData();
			callback(uuid, domains, {telemData.getTime(), data});
		} else {
			callback(uuid, domains, {});
		}
	};
	return addDeviceTelemetryCallback(id, telemtype_t::limit_switch, func); 
	*/
}

// ===========
// DEPRECATED:
// ===========

void initEncoder(devicegroup_t group, deviceserial_t serial, bool invertEncoder,
				 bool zeroEncoder, int32_t pulsesPerJointRev,
				 std::optional<std::chrono::milliseconds> telemetryPeriod) {
	auto groupCode = static_cast<uint8_t>(group);
	CANPacket p;
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

void emergencyStopMotors() {
	CANPacket p;
	AssembleGroupBroadcastingEmergencyStopPacket(&p, static_cast<uint8_t>(0x0),
												 ESTOP_ERR_GENERAL);
	can::sendCANPacket(p);
	std::this_thread::sleep_for(1000us);
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

void removeLimitSwitchCallback(callbackid_t id) {
	removeDeviceTelemetryCallback(id);
}
} // namespace can::motor
