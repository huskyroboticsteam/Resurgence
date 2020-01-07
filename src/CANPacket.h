#pragma once

#include <stdint.h>

typedef struct
{
    uint16_t id;
    uint8_t dlc;
    uint8_t data[8];
} CANPacket;

CANPacket ConstructCANPacket(uint16_t id, uint8_t dlc, uint8_t* data);
uint16_t ConstructCANID(uint8_t priority, uint8_t devGroup, uint8_t devSerial);

uint8_t ParseDataSenderDevice(uint8_t* data);
uint8_t ParseDataSenderSerial(uint8_t* data);
uint8_t ParseDataPayloadType(uint8_t* data);
uint8_t ParseDataSenderDeviceFromPacket(CANPacket *packet);
uint8_t ParseDataSenderSerialFromPacket(CANPacket *packet);
uint8_t ParseDataPayloadTypeFromPacket(CANPacket *packet);

// Device group nibbles
#define DEVICE_GROUP_BROADCAST          (uint8_t) 0x00
#define DEVICE_GROUP_RESERVED           (uint8_t) 0x01 // DO NOT USE. For future expansion
#define DEVICE_GROUP_JETSON             (uint8_t) 0x02
#define DEVICE_GROUP_POWER              (uint8_t) 0x03
#define DEVICE_GROUP_MOTOR_CONTROL      (uint8_t) 0x04
#define DEVICE_GROUP_TELEMETRY          (uint8_t) 0x05
#define DEVICE_GROUP_GPIO_BOARDS        (uint8_t) 0x06

// Common Mode Packet IDs
#define ID_COMMON_MODE_HIGH_NIBBLE      (uint8_t) DEVICE_GROUP_BROADCAST << 4
#define ID_ESTOP                        (uint8_t) ID_COMMON_MODE_HIGH_NIBBLE | 0x00
#define ID_HEARTBEAT                    (uint8_t) ID_COMMON_MODE_HIGH_NIBBLE | 0x01
#define ID_FAIL_REPORT                  (uint8_t) ID_COMMON_MODE_HIGH_NIBBLE | 0x02
#define ID_OVRD_PROTECTION              (uint8_t) ID_COMMON_MODE_HIGH_NIBBLE | 0x03
#define ID_TELEMETRY_TIMING             (uint8_t) ID_COMMON_MODE_HIGH_NIBBLE | 0x04
#define ID_TELEMETRY_PULL               (uint8_t) ID_COMMON_MODE_HIGH_NIBBLE | 0x05
#define ID_TELEMETRY_REPORT             (uint8_t) ID_COMMON_MODE_HIGH_NIBBLE | 0x06
#define ID_LED_COLOR                    (uint8_t) ID_COMMON_MODE_HIGH_NIBBLE | 0x07

// Motor Unit Packet IDs
#define ID_MOTOR_UNIT_HIGH_NIBBLE       (uint8_t) DEVICE_GROUP_MOTOR_CONTROL << 4
#define ID_MOTOR_UNIT_MODE_SEL          (uint8_t) ID_MOTOR_UNIT_HIGH_NIBBLE | 0x00
#define ID_MOTOR_UNIT_CHIP_TYPE_PULL    (uint8_t) ID_MOTOR_UNIT_HIGH_NIBBLE | 0x01
#define ID_MOTOR_UNIT_PWM_DIR_SET       (uint8_t) ID_MOTOR_UNIT_HIGH_NIBBLE | 0x02
#define ID_MOTOR_UNIT_PID_POS_TGT_SET   (uint8_t) ID_MOTOR_UNIT_HIGH_NIBBLE | 0x03
#define ID_MOTOR_UNIT_PID_P_SET         (uint8_t) ID_MOTOR_UNIT_HIGH_NIBBLE | 0x04
#define ID_MOTOR_UNIT_PID_I_SET         (uint8_t) ID_MOTOR_UNIT_HIGH_NIBBLE | 0x05
#define ID_MOTOR_UNIT_PID_D_SET         (uint8_t) ID_MOTOR_UNIT_HIGH_NIBBLE | 0x06
#define ID_MOTOR_UNIT_INIT              (uint8_t) ID_MOTOR_UNIT_HIGH_NIBBLE | 0x07
#define ID_MOTOR_UNIT_LIM_ALRT          (uint8_t) ID_MOTOR_UNIT_HIGH_NIBBLE | 0x08
#define ID_MOTOR_UNIT_ENC_PPJR_SET      (uint8_t) ID_MOTOR_UNIT_HIGH_NIBBLE | 0x09
#define ID_MOTOR_UNIT_MAX_JNT_REV_SET   (uint8_t) ID_MOTOR_UNIT_HIGH_NIBBLE | 0x0A

// GPIO Board Packet IDs
#define ID_GPIO_BOARD_HIGH_NIBBLE       (uint8_t) 0x20
// REST OF PACKETS TBD...

// Power Distribution Packet IDs
#define ID_POWER_DIST_HIGH_NIBBLE       (uint8_t) 0x30
// REST OF PACKETS TBD...

// Priority bits
#define PACKET_PRIORITY_HIGH            (uint8_t) 0x00
#define PACKET_PRIORITY_NORMAL          (uint8_t) 0x01

// Telemetry Types
#define PACKET_TELEMETRY_VOLTAGE        (uint8_t) 0x00
#define PACKET_TELEMETRY_CURRENT        (uint8_t) 0x01
#define PACKET_TELEMETRY_PWR_RAIL_STATE (uint8_t) 0x02
#define PACKET_TELEMETRY_TEMPERATURE    (uint8_t) 0x03
#define PACKET_TELEMETRY_ANG_POSITION   (uint8_t) 0x04
#define PACKET_TELEMETRY_GPS_LAT        (uint8_t) 0x05
#define PACKET_TELEMETRY_GPS_LON        (uint8_t) 0x06
#define PACKET_TELEMETRY_MAG_DIR        (uint8_t) 0x07
#define PACKET_TELEMETRY_ACCEL_X        (uint8_t) 0x08
#define PACKET_TELEMETRY_ACCEL_Y        (uint8_t) 0x09
#define PACKET_TELEMETRY_ACCEL_Z        (uint8_t) 0x0A
#define PACKET_TELEMETRY_GYRO_X         (uint8_t) 0x0B
#define PACKET_TELEMETRY_GYRO_Y         (uint8_t) 0x0C
#define PACKET_TELEMETRY_GYRO_Z         (uint8_t) 0x0D
#define PACKET_TELEMETRY_LIM_SW_STATE   (uint8_t) 0x0E

// ESTOP ERROR CODES
#define ESTOP_ERR_GENERAL               (uint8_t) 0x00
// MORE TBD...

