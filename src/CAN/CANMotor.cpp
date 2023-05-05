#include "CANMotor.h"

#include "CAN.h"
#include "CANUtils.h"

#include <chrono>
#include <cmath>
#include <thread>

extern "C" {
#include "../HindsightCAN/CANCommon.h"
#include "../HindsightCAN/CANMotorUnit.h"
#include "../HindsightCAN/CANPacket.h"
}

using namespace std::chrono_literals;
using robot::types::DataPoint;
using robot::types::LimitSwitchData;

namespace can::motor {

void emergencyStopMotors() {
	CANPacket p;
	AssembleGroupBroadcastingEmergencyStopPacket(&p, static_cast<uint8_t>(0x0),
												 ESTOP_ERR_GENERAL);
	can::sendCANPacket(p);
	std::this_thread::sleep_for(1000us);
}

void initMotor(deviceserial_t serial) {
	setMotorMode(serial, motormode_t::pwm);
	std::this_thread::sleep_for(1000us);
}

void setLimitSwitchLimits(deviceserial_t serial, int32_t lo, int32_t hi) {
	CANPacket p;
	auto motorGroupCode = static_cast<uint8_t>(devicegroup_t::motor);

	// 0 is low limit, 1 is high limit.
	AssembleLimSwEncoderBoundPacket(&p, motorGroupCode, serial, 0, lo);
	sendCANPacket(p);

	AssembleLimSwEncoderBoundPacket(&p, motorGroupCode, serial, 1, hi);
	sendCANPacket(p);
}

void setMotorPIDConstants(deviceserial_t serial, int32_t kP, int32_t kI, int32_t kD) {
	CANPacket p;
	auto motorGroupCode = static_cast<uint8_t>(devicegroup_t::motor);
	AssemblePSetPacket(&p, motorGroupCode, serial, kP);
	sendCANPacket(p);
	AssembleISetPacket(&p, motorGroupCode, serial, kI);
	sendCANPacket(p);
	AssembleDSetPacket(&p, motorGroupCode, serial, kD);
	sendCANPacket(p);
	std::this_thread::sleep_for(1000us);
}

void setMotorMode(deviceserial_t serial, motormode_t mode) {
	CANPacket p;
	AssembleModeSetPacket(&p, static_cast<uint8_t>(devicegroup_t::motor), serial,
						  static_cast<uint8_t>(mode));
	sendCANPacket(p);
	std::this_thread::sleep_for(1000us);
}

void setMotorPower(deviceserial_t serial, double power) {
	power = std::min(std::max(power, -1.0), 1.0);
	int powerInt = std::round(power * std::numeric_limits<int16_t>::max());
	int16_t dutyCycle = static_cast<int16_t>(powerInt);
	setMotorPower(serial, dutyCycle);
}

void setMotorPower(deviceserial_t serial, int16_t power) {
	CANPacket p;
	AssemblePWMDirSetPacket(&p, static_cast<uint8_t>(devicegroup_t::motor), serial, power);
	sendCANPacket(p);
}

void setMotorPIDTarget(deviceserial_t serial, int32_t target) {
	CANPacket p;
	AssemblePIDTargetSetPacket(&p, static_cast<uint8_t>(devicegroup_t::motor), serial, target);
	sendCANPacket(p);
}

void setPCAServo(deviceserial_t serial, uint8_t servoNum, int32_t angle) {
	CANPacket p;
	AssemblePCAServoPacket(&p, static_cast<uint8_t>(devicegroup_t::motor), serial, servoNum, angle);
	sendCANPacket(p);
}

DataPoint<int32_t> getMotorPosition(deviceserial_t serial) {
	return getDeviceTelemetry(std::make_pair(devicegroup_t::motor, serial),
							  telemtype_t::angle);
}

void pullMotorPosition(deviceserial_t serial) {
	pullDeviceTelemetry(std::make_pair(devicegroup_t::motor, serial), telemtype_t::angle);
}

callbackid_t addLimitSwitchCallback(
	deviceserial_t serial,
	const std::function<void(deviceserial_t serial,
							 DataPoint<LimitSwitchData> limitSwitchData)>& callback) {
	auto id = std::make_pair(devicegroup_t::motor, serial);
	// wrap callback in lambda to change signature
	auto func = [callback](deviceid_t deviceID, telemtype_t,
						   DataPoint<telemetry_t> telemData) {
		if (telemData) {
			LimitSwitchData data = telemData.getData();
			callback(deviceID.second, {telemData.getTime(), data});
		} else {
			callback(deviceID.second, {});
		}
	};
	return addDeviceTelemetryCallback(id, telemtype_t::limit_switch, func);
}

void removeLimitSwitchCallback(callbackid_t id) {
	removeDeviceTelemetryCallback(id);
}
} // namespace can::motor
