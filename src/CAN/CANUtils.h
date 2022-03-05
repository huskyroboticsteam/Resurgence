#pragma once

extern "C" {
#include "../HindsightCAN/CANCommon.h"
#include "../HindsightCAN/CANMotorUnit.h"
#include "../HindsightCAN/CANPacket.h"
}

#include <chrono>
#include <utility>

namespace can {

/**
 * @brief The possible device group codes.
 *
 * Every can device is in a group. These are the possible groups.
 */
enum class devicegroup_t {
	broadcast = DEVICE_GROUP_BROADCAST,
	reserved = DEVICE_GROUP_RESERVED,
	master = DEVICE_GROUP_MASTER,
	power = DEVICE_GROUP_POWER,
	motor = DEVICE_GROUP_MOTOR_CONTROL,
	telemetry = DEVICE_GROUP_TELEMETRY,
	gpio = DEVICE_GROUP_GPIO_BOARDS,
	science = DEVICE_GROUP_SCIENCE
};

/**
 * @brief The type of telemetries that devices can report.
 *
 * A device may offer some or none of these telemetry types.
 */
enum class telemtype_t {
	voltage = PACKET_TELEMETRY_VOLTAGE,
	current = PACKET_TELEMETRY_CURRENT,
	pwr_rail = PACKET_TELEMETRY_PWR_RAIL_STATE,
	temp = PACKET_TELEMETRY_TEMPERATURE,
	angle = PACKET_TELEMETRY_ANG_POSITION,
	gps_lat = PACKET_TELEMETRY_GPS_LAT,
	gps_lon = PACKET_TELEMETRY_GPS_LON,
	mag_dir = PACKET_TELEMETRY_MAG_DIR,
	accel_x = PACKET_TELEMETRY_ACCEL_X,
	accel_y = PACKET_TELEMETRY_ACCEL_Y,
	accel_z = PACKET_TELEMETRY_ACCEL_Z,
	gyro_x = PACKET_TELEMETRY_GYRO_X,
	gyro_y = PACKET_TELEMETRY_GYRO_Y,
	gyro_z = PACKET_TELEMETRY_GYRO_Z,
	limit_switch = PACKET_TELEMETRY_LIM_SW_STATE
	// TODO: add further telemetry types if required
};

/** @brief The type of the device serial number. */
using deviceserial_t = uint8_t;

/** @brief The type of telemetry data. */
using telemetry_t = int32_t;

/**
 * @brief A unique identifier for a CAN device.
 *
 * A CAN device is uniquely identified by its group code and its serial number.
 */
using deviceid_t = std::pair<devicegroup_t, deviceserial_t>;

/**
 * @brief Get the device group from the id of the given packet.
 *
 * @param packet The packet to extract the group from.
 * @return devicegroup_t The extracted device group.
 */
devicegroup_t getDeviceGroup(const CANPacket& packet);

/**
 * @brief Get the serial number from the id of the given packet.
 *
 * @param packet The packet to extract the serial number from.
 * @return deviceserial_t The extracted serial number.
 */
deviceserial_t getDeviceSerial(const CANPacket& packet);

/**
 * @brief Get the unique identifier from the id of the given packet.
 *
 * @param packet The packet ot extract the identifier from.
 * @return deviceid_t The extracted device identifier.
 */
deviceid_t getDeviceGroupAndSerial(const CANPacket& packet);

} // namespace can
