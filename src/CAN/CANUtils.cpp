#include "CANUtils.h"

#include <ios>
#include <sstream>

extern "C" {
// new
#include <CANDevices.h>
#include <CANPacket.h>

// old
#include <HindsightCAN/CANPacket.h>
}

namespace can {

uuid_t getUUIDFromPacket(const CANPacket_t& packet) {
	return packet.device.deviceUUID;
}

uuid_t getSenderUUID(const CANPacket_t& packet) {
	return packet.senderUUID;
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
