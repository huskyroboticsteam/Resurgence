#include "../Globals.h"
#include "json.hpp"

// Expected JSON format of Globals::status_data: each key is a string from one of the group lists below
// which identifies the device. The value is another JSON object containing one or more keys corresponding to
// telemetry measurements.

const std::string broadcast_group[] = {};
const std::string reserved_group[] = {};
const std::string master_group[] = {"master_broadcast", "jetson"};
const std::string power_group[] = {
    "power_broadcast",
    "battery",
    "chassis_main",
    "chassis_drive_l",
    "chassis_drive_r",
    "arm_1",
    "arm_2",
    "science"
};
const std::string motor_group[] = {
    "motor_broadcast",
    "arm_base",
    "shoulder",
    "elbow",
    "forearm",
    "diffleft",
    "diffright",
    "hand",
    "front_left_motor",
    "front_right_motor",
    "back_left_motor",
    "back_right_motor",
    "science"
};
const std::string telemetry_group[] = {"telemetry_broadcast", "localization", "imu", "temperature", "science"};
const std::string gpio_group[] = {"gpio_broadcast"};

const std::string *groups[] = {
    broadcast_group,
    reserved_group,
    master_group,
    power_group,
    motor_group,
    telemetry_group,
    gpio_group
};

const std::string telem_types[] = {
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
    return groups[device_group][device_serial];
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
