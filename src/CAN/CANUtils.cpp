#include "CANUtils.h"

#include <ios>
#include <sstream>

extern "C" {
#include <HindsightCAN/CANPacket.h>
}

namespace can {
// ===========
// UPDATED:
// ===========

uuid_t getUUIDFromPacket(const CANPacket& packet) {
    return (packet.id >> 3) & 0x7F;
}

domainmask_t getDomainsFromPacket(const CANPacket& packet) {
    return packet.id & 0x07;
}

bool deviceInDomain(const CANPacket& packet, domain_t domain) {
    return (getDomainsFromPacket(packet) & static_cast<uint8_t>(domain)) != 0;
}

uuid_t getSenderUUID(const CANPacket& packet) {
    return packet.data[1];
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
