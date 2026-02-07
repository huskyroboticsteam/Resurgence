#include "CANUtils.h"

#include <ios>
#include <sstream>

extern "C" {
// new
#include <CANPacket.h>
#include <CANDevices.h>

// old
#include <HindsightCAN/CANPacket.h>
}

namespace can {
// ===========
// UPDATED:
// ===========

uuid_t getDeviceFromPacket(const CANPacket_t& packet) {
	return packet.device.deviceUUID;
}

uuid_t getUUIDFromPacket(const CANPacket_t& packet) {
    return packet.device.deviceUUID;
}

uuid_t getSenderUUID(const CANPacket_t& packet) {
    return packet.senderUUID;
}

bool deviceInDomain(const CANPacket_t* packet, bool peripheralDomain, 
                           bool motorDomain, bool powerDomain) {
	if (peripheralDomain && packet->device.peripheralDomain) return true;
	if (motorDomain && packet->device.motorDomain) return true;
	if (powerDomain && packet->device.powerDomain) return true;
	return false;
}

bool isMotorDomain(const CANPacket_t* packet) {
	return packet->device.motorDomain;
}

bool isPeripheralDomain(const CANPacket_t* packet) {
	return packet->device.peripheralDomain;
}

bool isPowerDomain(const CANPacket_t* packet) {
	return packet->device.powerDomain;
}

// ===========
// DEPRECATED:
// ===========

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

deviceserial_t getSenderDeviceSerial(const CANPacket& packet) {
	return GetSenderDeviceSerialNumber(const_cast<CANPacket*>(&packet));
}

devicegroup_t getSenderDeviceGroup(const CANPacket& packet) {
	uint8_t groupCode = GetSenderDeviceGroupCode(const_cast<CANPacket*>(&packet));
	return static_cast<devicegroup_t>(groupCode);
}

deviceid_t getSenderDeviceGroupAndSerial(const CANPacket& packet) {
	return std::make_pair(getSenderDeviceGroup(packet), getSenderDeviceSerial(packet));
}

std::string packetToString(const CANPacket& packet) {
	std::stringstream ss;
	ss << std::hex << packet.id << "#";
	for (int i = 0; i < packet.dlc; i++) {
		ss << std::hex << static_cast<int>(packet.data[i]);
		if (i < packet.dlc - 1) {
			ss << ".";
		}
	}
	return ss.str();
}

} // namespace can
