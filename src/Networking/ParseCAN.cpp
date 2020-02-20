#include "../Globals.h"
#include "json.hpp"

// Expected JSON format of Globals::status_data: each key is a string from one of the group lists below
// which identifies the device. The value is another JSON object containing one or more keys corresponding to
// telemetry measurements.

extern const std::vector<std::string> broadcast_group = {};
extern const std::vector<std::string> reserved_group = {};
extern const std::vector<std::string> master_group = {"master_broadcast", "jetson"};
extern const std::vector<std::string> power_group = {
    "power_broadcast",
    "battery",
    "chassis_main",
    "chassis_drive_l",
    "chassis_drive_r",
    "arm_1",
    "arm_2",
    "science"
};
extern const std::vector<std::string> motor_group = {
    "motor_broadcast",
    "arm_base",
    "shoulder",
    "elbow",
    "forearm",
    "diffleft",
    "diffright",
    "hand",
    "front_left_wheel",
    "front_right_wheel",
    "back_left_wheel",
    "back_right_wheel",
    "science"
};
extern const std::vector<std::string> telemetry_group = {"telemetry_broadcast", "localization", "imu", "temperature", "science"};
extern const std::vector<std::string> gpio_group = {"gpio_broadcast"};

extern const std::vector<std::vector<std::string>> can_groups = {
    broadcast_group,
    reserved_group,
    master_group,
    power_group,
    motor_group,
    telemetry_group,
    gpio_group
};

extern const std::vector<std::string> telem_types = {
    "voltage",
    "current",
    "power_rail_state",
    "temperature",
    "angular_position",
    "gps_latitude",
    "gps_longitude",
    "magnetometer_direction",
    "accelerometer_x",
    "accelerometer_y",
    "accelerometer_z",
    "gyro_x",
    "gyro_y",
    "gyro_z",
    "limit_switch_status",
    "adc", // dunno what this means
    "gpio_state"
};

const std::string getDeviceTelemetryName(CANPacket &p) {
    uint8_t device_group = GetSenderDeviceGroupCode(&p);
    uint8_t device_serial = GetSenderDeviceSerialNumber(&p);
    return can_groups[device_group][device_serial];
}

void ParseCANPacket(CANPacket p)
{
    if (PacketIsOfID(&p, ID_TELEMETRY_REPORT)) {
        const std::string device_name = getDeviceTelemetryName(p);
        const std::string telem_type = telem_types[DecodeTelemetryType(&p)];
        // TODO is this data sometimes unsigned?
        Globals::status_data[device_name][telem_type] = DecodeTelemetryDataSigned(&p);
    }
}
