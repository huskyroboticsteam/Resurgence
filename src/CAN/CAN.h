#pragma once

#include <cstdint>
#include <optional>
#include <utility>

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
using telemtype_t = uint8_t; // TODO: convert to enum
using telemetry_t = int32_t;

using deviceid_t = std::pair<devicegroup_t, deviceserial_t>;

void initCAN();

void sendCANPacket(const CANPacket& packet);

std::optional<telemetry_t> getDeviceTelemetry(deviceid_t id, telemtype_t telemType);

} // namespace can
