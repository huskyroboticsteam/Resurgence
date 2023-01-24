#pragma once

#include "../utils/scheduler.h"
#include "../world_interface/data.h"
#include "CANUtils.h"

#include <functional>
#include <optional>

extern "C" {
#include <HindsightCAN/CANPacket.h>
}

/**
 * @namespace can
 * @brief Utilities for interacting with CAN devices.
 */
namespace can {

/**
 * @brief An ID for a telemetry callback.
 *
 * Users should not construct these themselves.
 */
using callbackid_t = std::tuple<deviceid_t, telemtype_t, uint32_t>;

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
 * @brief Get the latest telemetry from a CAN device.
 *
 * This method does NOT query for new data, it just returns the last reported value.
 *
 * @param id The device group and serial number of the device.
 * @param telemType The type of telemetry to get, as dictated by the specific device specs.
 * @return robot::types::DataPoint<telemetry_t> The telemetry value, with the timestamp of when
 * it was received. If no data is available for the given telemetry type, an empty data point
 * is returned.
 */
robot::types::DataPoint<telemetry_t> getDeviceTelemetry(deviceid_t id, telemtype_t telemType);

/**
 * @brief Ping the given CAN device to send the given telemetry data.
 *
 * The CAN device will asynchronously send the new data in an unspecified amount of time.
 * Not all CAN devices may support pulling telemetry.
 *
 * @param id The device group and serial number of the device.
 * @param telemType The type of telemetry to get, as dictated by the specific device specs.
 */
void pullDeviceTelemetry(deviceid_t id, telemtype_t telemType);

/**
 * @brief Periodically pull the latest telemetry data from the specified CAN device
 * asychronously.
 *
 * This method is NOT thread safe.
 *
 * @param id The device group and serial number of the device.
 * @param telemType The type of telemetry to get, as dictated by the specific device specs.
 * @param period The period to wait in between sending pull requests.
 */
void scheduleTelemetryPull(deviceid_t id, telemtype_t telemType,
						   std::chrono::milliseconds period);

/**
 * @brief Stop pulling the latest telemetry data from the given device.
 *
 * This method is NOT thread safe.
 *
 * @param id The device group and serial number of the device.
 * @param telemType The type of telemetry to get, as dictated by the specific device specs.
 */
void unscheduleTelemetryPull(deviceid_t id, telemtype_t telemType);

/**
 * @brief Stop pulling the latest telemetry data from all currently scheduled devices.
 *
 * This method is NOT thread safe.
 */
void unscheduleAllTelemetryPulls();

/**
 * @brief Add a callback which is invoked when data is recieved.
 *
 * The callback is invoked when telemetry data of the given type is received
 * from the given device.
 *
 * @param id The ID of the device the callback is listening for.
 * @param telemType The type of telemetry the callback is listening for.
 * @param callback The callback that will be invoked with the device ID, telemetry type, and
 * the telemetry data.
 * @return callbackid_t A callback ID, which can be used with removeDeviceTelemetryCallback()
 * to remove a callback.
 */
callbackid_t addDeviceTelemetryCallback(
	deviceid_t id, telemtype_t telemType,
	const std::function<void(deviceid_t, telemtype_t, robot::types::DataPoint<telemetry_t>)>&
		callback);

/**
 * @brief Remove a previously registered telemetry callback.
 *
 * The callback associated with the given callback ID is removed.
 *
 * @param id A callback ID which was previously returned by addDeviceTelemetryCallback().
 */
void removeDeviceTelemetryCallback(callbackid_t id);

} // namespace can
