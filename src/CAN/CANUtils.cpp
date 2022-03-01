#include "CANUtils.h"

#include "CAN.h"

#include <chrono>
#include <map>
#include <thread>

extern "C" {
#include "../HindsightCAN/CANCommon.h"
#include "../HindsightCAN/CANMotorUnit.h"
#include "../HindsightCAN/CANPacket.h"
}

using namespace std::chrono_literals;

namespace can {
devicegroup_t getDeviceGroup(const CANPacket& packet) {
	uint8_t groupCode = GetDeviceGroupCode(const_cast<CANPacket*>(&packet));
	return static_cast<devicegroup_t>(groupCode);
}

deviceserial_t getDeviceSerial(const CANPacket& packet) {
	return GetDeviceSerialNumber(const_cast<CANPacket*>(&packet));
}

deviceid_t getDeviceGroupAndSerial(const CANPacket& packet) {
	return std::make_pair(getDeviceGroup(packet), getDeviceSerial(packet));
}

void initMotor(deviceserial_t serial) {
	setMotorMode(serial, motormode_t::pwm);
}

void initMotor(deviceserial_t serial, bool invertEncoder, bool zeroEncoder,
			   int32_t pulsesPerJointRev, std::chrono::milliseconds telemetryPeriod) {
	initMotor(serial);
	auto motorGroupCode = static_cast<uint8_t>(devicegroup_t::motor);
	CANPacket p;
	AssembleEncoderInitializePacket(&p, motorGroupCode, serial, 0, invertEncoder, zeroEncoder);
	sendCANPacket(p);
	std::this_thread::sleep_for(1000us);
	AssembleEncoderPPJRSetPacket(&p, motorGroupCode, serial, pulsesPerJointRev);
	sendCANPacket(p);
	std::this_thread::sleep_for(1000us);
	AssembleTelemetryTimingPacket(&p, motorGroupCode, serial, PACKET_TELEMETRY_ANG_POSITION,
								  telemetryPeriod.count());
	sendCANPacket(p);
	std::this_thread::sleep_for(1000us);
}

void initMotor(deviceserial_t serial, bool invertEncoder, bool zeroEncoder,
			   int32_t pulsesPerJointRev, std::chrono::milliseconds telemetryPeriod,
			   int32_t kP, int32_t kI, int32_t kD) {
	initMotor(serial, invertEncoder, zeroEncoder, pulsesPerJointRev, telemetryPeriod);
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
} // namespace can
