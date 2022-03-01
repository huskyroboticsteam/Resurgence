#pragma once

#include <cstdint>
#include <optional>

extern "C" {
#include "../HindsightCAN/CANPacket.h"
}

namespace can {

enum class devicegroup_t {
	broadcast,
	reserved,
	master,
	power,
	motor,
	telemetry,
	gpio,
	science
};

using deviceserial_t = uint8_t;
using telemtype_t = uint8_t;

using deviceid_t = std::pair<devicegroup_t, deviceserial_t>;

void initCAN();

void sendCANPacket(const CANPacket& packet);

std::optional<int32_t> getDeviceTelemetry(deviceid_t id, telemtype_t telemType);

} // namespace can
