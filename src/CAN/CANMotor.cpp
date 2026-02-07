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
	auto motorGroupCode = static_cast<uint8_t>(group);
	AssemblePSetPacket(&p, motorGroupCode, serial, kP);
	sendCANPacket(p);
	std::this_thread::sleep_for(1ms);
	AssembleISetPacket(&p, motorGroupCode, serial, kI);
	sendCANPacket(p);
	std::this_thread::sleep_for(1ms);
	AssembleDSetPacket(&p, motorGroupCode, serial, kD);
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

void setServoPos(devicegroup_t group, deviceserial_t serial, uint8_t servoNum, int32_t angle) {
	CANPacket p;
	AssembleScienceServoPacket(&p, static_cast<uint8_t>(group), serial, servoNum, angle);
	sendCANPacket(p);
}

void setStepperTurnAngle(devicegroup_t group, deviceserial_t serial, uint8_t stepper, int16_t angle) {
  CANPacket p;
  AssembleScienceStepperTurnAnglePacket(&p, static_cast<uint8_t>(group), serial, stepper, angle, 0x3);
  sendCANPacket(p);
}

void setLED(devicegroup_t group, deviceserial_t serial, uint8_t LED, uint8_t value) {
  CANPacket p;
  uint8_t data[3];
  data[0] = 0xFA;
  data[1] = LED;
  data[2] = value;
  AssembleCANPacket(&p, 0x1, static_cast<uint8_t>(group), serial, 0xFA, 3, data);
  sendCANPacket(p);
}

void setActuator(devicegroup_t group, deviceserial_t serial, uint8_t value) {
  CANPacket p;
  uint8_t data[3];
  data[0] = 0x13;
  data[1] = 2;
  data[2] = (value == 1) ? 1 : 255;
  AssembleCANPacket(&p, 0x1, static_cast<uint8_t>(group), serial, 0x13, 3, data);
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
