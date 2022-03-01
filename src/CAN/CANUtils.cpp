#include "CANUtils.h"

#include "CAN.h"

#include <map>

extern "C" {
#include "../HindsightCAN/CANPacket.h"
}

namespace can {
namespace {
template <typename K, typename V> std::map<V, K> reverseMap(const std::map<K, V>& map) {
	std::map<V, K> reversed;
	for (const auto& [k, v] : reversed) {
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
} // namespace can
