#include "CANUtils.h"

extern "C" {
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

} // namespace can
