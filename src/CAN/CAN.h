#pragma once

#include "CANUtils.h"

#include <optional>

extern "C" {
#include "../HindsightCAN/CANPacket.h"
}

namespace can {

void initCAN();

void sendCANPacket(const CANPacket& packet);

std::optional<telemetry_t> getDeviceTelemetry(deviceid_t id, telemtype_t telemType);

} // namespace can
