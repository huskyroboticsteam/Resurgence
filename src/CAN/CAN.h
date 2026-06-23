#pragma once

#include "../Constants.h"
#include "../utils/scheduler.h"
#include "../world_interface/data.h"

#include <functional>
#include <optional>
#include <shared_mutex>

#include <linux/can.h>
#include <nlohmann/json.hpp>

extern "C" {
#include <CAN26.h>
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
enum class axis_state_t : uint8_t {
	idle = BLDC_AXIS_IDLE,
	full_calib = BLDC_AXIS_FULL_CALIBRATION_SEQUENCE,
	motor_calib = BLDC_AXIS_MOTOR_CALIBRATION,
	encoder_offset_calib = BLDC_AXIS_ENCODER_OFFSET_CALIBRATION,
	closed_loop_control = BLDC_AXIS_CLOSED_LOOP_CONTROL,
	lockin_spin = BLDC_AXIS_LOCKIN_SPIN,
};

} // namespace motor

enum class led_t : uint8_t {
	red,
	green,
	blue,
};

/** @brief ODrive endpoint ID */
using endpointid_t = uint16_t;

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

/**
 * @brief Print a CAN packet.
 *
 * @param packet The CAN packet to print.
 */
void printCANPacket(const CANPacket_t& packet);

/**
 * @brief Broadcasts an emergency stop packet.
 */
void emergencyStop();

/**
 * @brief Add a callback to run when we receive a read result packet corresponding
 * to the input endpoint. Callbacks persist until they are manually removed using
 * removeDirectReadCallback()
 *
 * This method is thread-safe.
 *
 * @param device The CAN device associated with this read.
 * @param endpoint The endpoint to respond to.
 * @param callback The function to call when we receive data, called with the decoded packet.
 */
void addDirectReadCallback(
	CANDevice_t device, endpointid_t endpoint_id,
	const std::function<void(CANMotorPacket_BLDC_DirectReadResult_Decoded_t,
							 std::unique_lock<std::shared_mutex>)>& callback);

/**
 * @brief Removes a callback.
 * Does nothing if callback does not exist.
 *
 * This method is thread-safe.
 *
 * @param device The CAN device associated with the read callback.
 * @param endpoint The endpoint to remove the callback for.
 */
void removeDirectReadCallback(CANDevice_t device, endpointid_t endpoint);

/**
 * @brief Retrieves a JSON that corresponds to the endpoint name input.
 *
 * @param boardid The ID of the board to get the endpoint for. This is used
 * to determine whether to fetch S1 or Pro endpoints.
 * @param endpoint The name of the endpoint to retrieve.
 */
nlohmann::json getEndpoint(robot::types::boardid_t boardid, std::string endpoint);

void setLED(led_t led);

} // namespace can
