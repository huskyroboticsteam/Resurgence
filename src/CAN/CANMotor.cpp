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
}

using namespace std::chrono_literals;
using robot::types::DataPoint;
using robot::types::LimitSwitchData;

namespace can::motor {

void initEncoder(deviceid_t encoder, bool invertEncoder, bool zeroEncoder,
				 int32_t pulsesPerJointRev,
				 std::optional<std::chrono::milliseconds> telemetryPeriod) {
	uint8_t motorGroupCode = static_cast<uint8_t>(encoder.first);
	uint8_t serial = encoder.second;
	CANPacket p;
	AssembleEncoderInitializePacket(&p, motorGroupCode, serial, sensor_t::encoder,
									invertEncoder, zeroEncoder);
	sendCANPacket(p);
	std::this_thread::sleep_for(1000us);
	AssembleEncoderPPJRSetPacket(&p, motorGroupCode, serial, pulsesPerJointRev);
	sendCANPacket(p);
	std::this_thread::sleep_for(1000us);
	if (telemetryPeriod) {
		scheduleTelemetryPull(encoder, telemtype_t::angle, telemetryPeriod.value());
	}
}

void initPotentiometer(deviceid_t pot, int32_t posLo, int32_t posHi, uint16_t adcLo,
					   uint16_t adcHi,
					   std::optional<std::chrono::milliseconds> telemetryPeriod) {
	CANPacket packet;
	uint8_t motorGroupCode = static_cast<uint8_t>(pot.first);
	uint8_t serial = pot.second;
	AssemblePotHiSetPacket(&packet, motorGroupCode, serial, adcHi, posHi);
	sendCANPacket(packet);
	std::this_thread::sleep_for(1ms);
	AssemblePotLoSetPacket(&packet, motorGroupCode, serial, adcLo, posLo);
	sendCANPacket(packet);
	if (telemetryPeriod) {
		scheduleTelemetryPull(pot, telemtype_t::angle, telemetryPeriod.value());
	}
}

void emergencyStopMotors() {
	CANPacket p;
	AssembleGroupBroadcastingEmergencyStopPacket(&p, static_cast<uint8_t>(0x0),
												 ESTOP_ERR_GENERAL);
	can::sendCANPacket(p);
	std::this_thread::sleep_for(1000us);
}

void initMotor(deviceid_t motor) {
	setMotorMode(motor, motormode_t::pwm);
	std::this_thread::sleep_for(1000us);
}

void setLimitSwitchLimits(deviceserial_t serial, int32_t lo, int32_t hi) {
	CANPacket p;
	uint8_t motorGroupCode = static_cast<uint8_t>(devicegroup_t::motor);

	// 0 is low limit, 1 is high limit.
	AssembleLimSwEncoderBoundPacket(&p, motorGroupCode, serial, 0, lo);
	sendCANPacket(p);
	std::this_thread::sleep_for(1ms);
	AssembleLimSwEncoderBoundPacket(&p, motorGroupCode, serial, 1, hi);
	sendCANPacket(p);
}

void setMotorPIDConstants(deviceserial_t serial, int32_t kP, int32_t kI, int32_t kD) {
	CANPacket p;
	uint8_t motorGroupCode = static_cast<uint8_t>(devicegroup_t::motor);
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

void setMotorMode(deviceid_t motor, motormode_t mode) {
	CANPacket p;
	uint8_t motorGroupCode = static_cast<uint8_t>(motor.first);
	uint8_t serial = motor.second;
	AssembleModeSetPacket(&p, motorGroupCode, serial, static_cast<uint8_t>(mode));
	sendCANPacket(p);
	std::this_thread::sleep_for(1000us);
}

void setMotorPower(deviceid_t motor, double power) {
	power = std::min(std::max(power, -1.0), 1.0);
	int powerInt = std::round(power * std::numeric_limits<int16_t>::max());
	int16_t dutyCycle = static_cast<int16_t>(powerInt);
	setMotorPower(motor, dutyCycle);
}

void setMotorPower(deviceid_t motor, int16_t power) {
	CANPacket p;
	uint8_t motorGroupCode = static_cast<uint8_t>(motor.first);
	uint8_t serial = motor.second;
	AssemblePWMDirSetPacket(&p, motorGroupCode, serial, power);
	sendCANPacket(p);
}

void setMotorPIDTarget(deviceserial_t serial, int32_t target) {
	CANPacket p;
	AssemblePIDTargetSetPacket(&p, static_cast<uint8_t>(devicegroup_t::motor), serial, target);
	sendCANPacket(p);
}

void setServoPos(deviceserial_t serial, uint8_t servoNum, int32_t angle) {
	CANPacket p;
	AssemblePCAServoPacket(&p, static_cast<uint8_t>(devicegroup_t::motor), serial, servoNum,
						   angle);
	sendCANPacket(p);
}

DataPoint<int32_t> getMotorPosition(deviceid_t motor) {
	return getDeviceTelemetry(motor, telemtype_t::angle);
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
