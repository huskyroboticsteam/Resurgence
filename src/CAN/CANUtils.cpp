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
namespace {
template <typename K, typename V> std::map<V, K> reverseMap(const std::map<K, V>& map) {
	std::map<V, K> reversed;
	for (const auto& [k, v] : map) {
		reversed.emplace(v, k);
	}
	return reversed;
}

const std::map<uint8_t, devicegroup_t> codeToGroupMap = {
	{DEVICE_GROUP_BROADCAST, devicegroup_t::broadcast},
	{DEVICE_GROUP_RESERVED, devicegroup_t::reserved},
	{DEVICE_GROUP_MASTER, devicegroup_t::master},
	{DEVICE_GROUP_POWER, devicegroup_t::power},
	{DEVICE_GROUP_MOTOR_CONTROL, devicegroup_t::motor},
	{DEVICE_GROUP_TELEMETRY, devicegroup_t::telemetry},
	{DEVICE_GROUP_GPIO_BOARDS, devicegroup_t::gpio},
	{DEVICE_GROUP_SCIENCE, devicegroup_t::science}};

const std::map<devicegroup_t, uint8_t> groupToCodeMap = reverseMap(codeToGroupMap);
} // namespace
devicegroup_t getDeviceGroup(const CANPacket& packet) {
	uint8_t groupCode = GetDeviceGroupCode(const_cast<CANPacket*>(&packet));
	return getDeviceGroup(groupCode);
}

devicegroup_t getDeviceGroup(uint8_t groupCode) {
	return codeToGroupMap.at(groupCode);
}

uint8_t getDeviceGroupCode(devicegroup_t group) {
	return groupToCodeMap.at(group);
}

deviceserial_t getDeviceSerial(const CANPacket& packet) {
	return GetDeviceSerialNumber(const_cast<CANPacket*>(&packet));
}

deviceid_t getDeviceGroupAndSerial(const CANPacket& packet) {
	return std::make_pair(getDeviceGroup(packet), getDeviceSerial(packet));
}

void initMotor(deviceserial_t serial) {
	CANPacket p;
	AssembleModeSetPacket(&p, getDeviceGroupCode(devicegroup_t::motor), serial,
						  MOTOR_UNIT_MODE_PWM);
	sendCANPacket(p);
	std::this_thread::sleep_for(1000us);
}

void initMotor(deviceserial_t serial, bool invertEncoder, bool zeroEncoder,
			   int32_t pulsesPerJointRev, std::chrono::milliseconds telemetryPeriod) {
	initMotor(serial);
	auto motorGroupCode = getDeviceGroupCode(devicegroup_t::motor);
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
	auto motorGroupCode = getDeviceGroupCode(devicegroup_t::motor);
	AssemblePSetPacket(&p, motorGroupCode, serial, kP);
	sendCANPacket(p);
	AssembleISetPacket(&p, motorGroupCode, serial, kI);
	sendCANPacket(p);
	AssembleDSetPacket(&p, motorGroupCode, serial, kD);
	sendCANPacket(p);
	std::this_thread::sleep_for(1000us);
}
} // namespace can
