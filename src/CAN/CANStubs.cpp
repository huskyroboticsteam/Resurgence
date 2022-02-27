
#include "CAN.h"

namespace can {
void initCAN() {}

void sendCANPacket(const CANPacket& packet) {}

std::optional<int32_t> getDeviceTelemetry(devicegroup_t group, deviceserial_t serial,
										  telemtype_t telemType) {}
} // namespace can