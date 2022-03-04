
#include "CAN.h"

namespace can {
void initCAN() {}

void sendCANPacket(const CANPacket& packet) {}

robot::types::DataPoint<telemetry_t> getDeviceTelemetry(deviceid_t id, telemtype_t telemType) {
	return {};
}
} // namespace can