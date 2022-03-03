#pragma once

#include "CANUtils.h"
#include "../world_interface/data.h"

#include <optional>

extern "C" {
#include "../HindsightCAN/CANPacket.h"
}

namespace can {

void initCAN();

void sendCANPacket(const CANPacket& packet);

robot::types::DataPoint<telemetry_t> getDeviceTelemetry(deviceid_t id, telemtype_t telemType);

} // namespace can
