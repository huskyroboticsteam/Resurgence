
#include "CAN.h"
#include <iostream>

namespace can {
void initCAN() {}

void sendCANPacket(const CANPacket& packet) {
	std::cout << "Packet: " << packetToString(packet) << std::endl;
}

robot::types::DataPoint<telemetry_t> getDeviceTelemetry(deviceid_t id, telemtype_t telemType) {
	return {};
}
} // namespace can