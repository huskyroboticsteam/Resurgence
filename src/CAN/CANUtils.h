#pragma once

extern "C" {
#include "../HindsightCAN/CANPacket.h"
#include "../HindsightCAN/CANMotorUnit.h"
#include "../HindsightCAN/CANCommon.h"
}

#include <chrono>
#include <utility>

namespace can {
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

enum class motormode_t {
	pwm = MOTOR_UNIT_MODE_PWM,
	pid = MOTOR_UNIT_MODE_PID
};

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

using deviceserial_t = uint8_t;
using telemetry_t = int32_t;

using deviceid_t = std::pair<devicegroup_t, deviceserial_t>;

devicegroup_t getDeviceGroup(const CANPacket& packet);

deviceserial_t getDeviceSerial(const CANPacket& packet);

deviceid_t getDeviceGroupAndSerial(const CANPacket& packet);

} // namespace can