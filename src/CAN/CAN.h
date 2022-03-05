#pragma once

#include "../world_interface/data.h"
#include "CANUtils.h"

#include <optional>

extern "C" {
#include "../HindsightCAN/CANPacket.h"
}

/**
 * @namespace can
 * @brief Utilities for interacting with CAN devices.
 */
namespace can {

/**
 * @brief Initialize the CAN interface.
 *
 * This should only be called once.
 *
 * @note If CAN initialization fails, the program will exit.
 */
void initCAN();

/**
 * @brief Send a CAN packet.
 *
 * No further formatting is done on the packet.
 * This method is thread-safe.
 *
 * @param packet The CAN packet to send.
 */
void sendCANPacket(const CANPacket& packet);

/**
 * @brief Get telemetry from a CAN device.
 *
 * @param id The device group and serial number of the device.
 * @param telemType The type of telemetry to get, as dictated by the specific device specs.
 * @return robot::types::DataPoint<telemetry_t> The telemetry value, with the timestamp of when
 * it was received. If no data is available for the given telemetry type, an empty data point
 * is returned.
 */
robot::types::DataPoint<telemetry_t> getDeviceTelemetry(deviceid_t id, telemtype_t telemType);

} // namespace can
