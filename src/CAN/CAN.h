#pragma once

#include "CANUtils.h"
#include "../Constants.h"
#include "../utils/scheduler.h"
#include "../world_interface/data.h"

#include <functional>
#include <optional>

#include <linux/can.h>
#include <nlohmann/json.hpp>

extern "C" {
#include <CANDevices.h>
#include <CANPacket.h>

#include <Packets/Motor.h>
#include <Packets/Peripheral.h>

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
enum class axis_state_t : uint8_t {
	idle = BLDC_AXIS_IDLE,
	full_calib = BLDC_AXIS_FULL_CALIBRATION_SEQUENCE,
	motor_calib = BLDC_AXIS_MOTOR_CALIBRATION,
	encoder_offset_calib = BLDC_AXIS_ENCODER_OFFSET_CALIBRATION,
	closed_loop_control = BLDC_AXIS_CLOSED_LOOP_CONTROL,
	lockin_spin = BLDC_AXIS_LOCKIN_SPIN,
};
} // namespace motor

/**
 * @brief An ID for a telemetry callback.
 *
 * Users should not construct these themselves.
 */
using callbackid_t = std::tuple<uuid_t, telemtype_t, uint32_t>;

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
void addDirectReadCallback(CANDevice_t device, uint16_t endpoint, const std::function<void(CANMotorPacket_BLDC_DirectReadResult_Decoded_t)>& callback);

/**
 * @brief Removes a callback.
 *
 * This method is thread-safe.
 *
 * @param device The CAN device associated with the read callback.
 * @param endpoint The endpoint to remove the callback for.
 */
void removeDirectReadCallback(CANDevice_t device, uint16_t endpoint);

/**
 * @brief Retrieves a JSON that corresponds to the endpoint name input.
 *
 * @param boardid The ID of the board to get the endpoint for. This is used
 * to determine whether to fetch S1 or Pro endpoints.
 * @param endpoint The name of the endpoint to retrieve.
 */
nlohmann::json getEndpoint(boardid_t boardid, std::string endpoint);

} // namespace can
