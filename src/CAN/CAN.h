#pragma once

#include "../utils/scheduler.h"
#include "../world_interface/data.h"
#include "CANUtils.h"
#include "../Constants.h"
#include <functional>
#include <optional>
#include <linux/can.h>

extern "C" {
#include <CANDevices.h>
#include <CANPacket.h>

#include <Packets/Motor.h>

#include <Packets/DecodeMotor.h>
#include <Packets/DecodePeripheral.h>
#include <Packets/DecodePower.h>
#include <Packets/DecodeUniversal.h>
}

/**
 * @namespace can
 * @brief Utilities for interacting with CAN devices.
 */
namespace can {

namespace motor {

/** @brief ODrive Control Modes */
enum class control_mode_t : uint8_t {
	position = BLDC_POSITION_CONTROL,
	velocity = BLDC_VELOCITY_CONTROL,
};

/** @brief ODrive Input Modes */
enum class input_mode_t : uint8_t {
	passthrough = BLDC_PASSTHROUGH_INPUT,
	vel_ramp = BLDC_VEL_RAMP_INPUT,
};

/** @brief ODrive Axis States */
enum class axis_state_t : uint32_t {
	idle = BLDC_AXIS_IDLE,
	full_calib = BLDC_AXIS_FULL_CALIBRATION_SEQUENCE,
	motor_calib = BLDC_AXIS_MOTOR_CALIBRATION,
	encoder_offset_calib = BLDC_AXIS_ENCODER_OFFSET_CALIBRATION,
	closed_loop_control = BLDC_AXIS_CLOSED_LOOP_CONTROL,
	lockin_spin = BLDC_AXIS_LOCKIN_SPIN,
};

/** @brief ODrive endpoints */
constexpr auto ENDPOINTS = frozen::make_unordered_map<frozen::string, uint16_t>({
	{"axis0.current_state", 232}, // uint8, r
	{"axis0.requested_state", 233}, // uint8, rw
	{"axis0.config.enable_watchdog", 254}, // bool, rw
	{"axis0.config.general_lockin.vel", 278}, // float, rw
	{"axis0.controller.input_vel", 374}, // float, rw
	{"axis0.controller.config.control_mode", 389}, // uint8, rw
	{"axis0.controller.confg.input_mode", 390}, // uint8, rw
	{"axis0.controller.config.vel_limit", 396}, // float, rw
});

} // namespace motor

/**
 * @brief An ID for a telemetry callback.
 *
 * Users should not construct these themselves.
 */
using callbackid_t = std::tuple<uuid_t, telemtype_t, uint32_t>;
// using callbackid_t = std::tuple<deviceid_t, telemtype_t, uint32_t>;

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
void sendCANPacket(const CANPacket_t& packet);
bool sendCANFrame(const canfd_frame& frame);

/**
 * @brief Print a CAN packet.
 *
 * @param packet The CAN packet to print.
 */
void printCANPacket(const CANPacket_t& packet);

void initHeartbeatWatchdog();
void handleHeartbeatPacket(CANPacket_t& packet);

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
robot::types::DataPoint<telemetry_t> getDeviceTelemetry(uuid_t uuid, telemtype_t telemType);

/**
 * @brief Ping the given CAN device to send the given telemetry data.
 *
 * The CAN device will asynchronously send the new data in an unspecified amount of time.
 * Not all CAN devices may support pulling telemetry.
 *
 * @param id The device group and serial number of the device.
 * @param telemType The type of telemetry to get, as dictated by the specific device specs.
 */
void pullDeviceTelemetry(uuid_t uuid, telemtype_t telemType);

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
void scheduleTelemetryPull(uuid_t uuid, telemtype_t telemType,
						   std::chrono::milliseconds period);

/**
 * @brief Stop pulling the latest telemetry data from the given device.
 *
 * This method is NOT thread safe.
 *
 * @param id The device group and serial number of the device.
 * @param telemType The type of telemetry to get, as dictated by the specific device specs.
 */
void unscheduleTelemetryPull(uuid_t uuid, telemtype_t telemType);

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
	CANDeviceUUID_t uuid, telemtype_t telemType,
	const std::function<void(CANDeviceUUID_t, telemtype_t, robot::types::DataPoint<telemetry_t>)>&
		callback);

/**
 * @brief Remove a previously registered telemetry callback.
 *
 * The callback associated with the given callback ID is removed.
 *
 * @param id A callback ID which was previously returned by addDeviceTelemetryCallback().
 */
void removeDeviceTelemetryCallback(callbackid_t id);

void addDirectReadCallback(CANDevice_t device, uint16_t endpoint, const std::function<void(CANMotorPacket_BLDC_DirectReadResult_Decoded_t)>& callback);

} // namespace can
