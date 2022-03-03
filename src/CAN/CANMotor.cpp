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

namespace can::motor {

void emergencyStopMotors() {
	CANPacket p;
	AssembleGroupBroadcastingEmergencyStopPacket(
		&p, static_cast<uint8_t>(devicegroup_t::motor), ESTOP_ERR_GENERAL);
	can::sendCANPacket(p);
	std::this_thread::sleep_for(1000us);
}

void initMotor(deviceserial_t serial) {
	setMotorMode(serial, motormode_t::pwm);
	std::this_thread::sleep_for(1000us);
}

void initMotor(deviceserial_t serial, bool invertEncoder, bool zeroEncoder,
			   int32_t pulsesPerJointRev, std::chrono::milliseconds telemetryPeriod,
			   int32_t kP, int32_t kI, int32_t kD) {
	initMotor(serial, invertEncoder, zeroEncoder, pulsesPerJointRev, telemetryPeriod);
	setMotorPIDConstants(serial, kP, kI, kD);
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
	int powerInt = std::round(power * ((1 << 16) - 1));
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

robot::types::DataPoint<int32_t> getMotorPosition(deviceserial_t serial) {
	return getDeviceTelemetry(std::make_pair(devicegroup_t::motor, serial),
							  telemtype_t::angle);
}
} // namespace can::motor