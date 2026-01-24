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

uint16_t constructCAN(uint8_t priority, uuid_t uuid, domainmask_t domains) {
	uint16_t CANID = 0x0000;
	CANID |= ((priority & 0x01) << 10); // bit 10
	CANID |= ((uuid & 0x7F) << 3); // bit 9-3
	CANID |= ((domains & 0x07)); // bit 2-0
	return CANID;
}

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

int writeCANHeader(uint8_t* data, uint8_t commandID, uuid_t senderUUID, bool requestACK) {
    data[0] = requestACK ? (commandID | 0x80) : (commandID & 0x7F);
    data[1] = senderUUID;
    return 2;
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
