#pragma once

#include <stdint.h>

typedef struct CANPacket
{
    uint16_t id;
    uint8_t dlc;
    uint8_t data[8];
} CANPacket;

uint16_t ConstructCANID(uint8_t priority, uint8_t devGroup, uint8_t devSerial);
CANPacket ConstructCANPacket(uint16_t id, uint8_t dlc, uint8_t data[8]);
uint8_t ParseDataSenderDevice(uint8_t data[8]);
uint8_t ParseDataSenderSerial(uint8_t data[8]);
uint8_t ParseDataPayloadType(uint8_t data[8]);

// Common Mode Packet IDs
#define ID_COMMON_MODE_HIGH_NIBBLE      (uint8_t) 0x00;
#define ID_ESTOP                        (uint8_t) ID_COMMON_MODE_HIGH_NIBBLE | 0x00;
#define ID_HEARTBEAT                    (uint8_t) ID_COMMON_MODE_HIGH_NIBBLE | 0x01;
#define ID_FAIL_REPORT                  (uint8_t) ID_COMMON_MODE_HIGH_NIBBLE | 0x02;
#define ID_OVRD_PROTECTION              (uint8_t) ID_COMMON_MODE_HIGH_NIBBLE | 0x03;
#define ID_TELEMETRY_TIMING             (uint8_t) ID_COMMON_MODE_HIGH_NIBBLE | 0x04;
#define ID_TELEMETRY_PULL               (uint8_t) ID_COMMON_MODE_HIGH_NIBBLE | 0x05;
#define ID_TELEMETRY_REPORT             (uint8_t) ID_COMMON_MODE_HIGH_NIBBLE | 0x06;
#define ID_LED_COLOR                    (uint8_t) ID_COMMON_MODE_HIGH_NIBBLE | 0x07;

// Motor Unit Packet IDs
#define ID_MOTOR_UNIT_HIGH_NIBBLE       (uint8_t) 0x01;
#define ID_MOTOR_UNIT_MODE_SEL          (uint8_t) ID_MOTOR_UNIT_HIGH_NIBBLE | 0x00;
#define ID_MOTOR_UNIT_CHIP_TYPE_PULL    (uint8_t) ID_MOTOR_UNIT_HIGH_NIBBLE | 0x01;
#define ID_MOTOR_UNIT_PWM_DIR_SET       (uint8_t) ID_MOTOR_UNIT_HIGH_NIBBLE | 0x02;
#define ID_MOTOR_UNIT_PID_POS_TGT_SET   (uint8_t) ID_MOTOR_UNIT_HIGH_NIBBLE | 0x03;
#define ID_MOTOR_UNIT_PID_P_SET         (uint8_t) ID_MOTOR_UNIT_HIGH_NIBBLE | 0x04;
#define ID_MOTOR_UNIT_PID_I_SET         (uint8_t) ID_MOTOR_UNIT_HIGH_NIBBLE | 0x05;
#define ID_MOTOR_UNIT_PID_D_SET         (uint8_t) ID_MOTOR_UNIT_HIGH_NIBBLE | 0x06;
#define ID_MOTOR_UNIT_INIT              (uint8_t) ID_MOTOR_UNIT_HIGH_NIBBLE | 0x07;
#define ID_MOTOR_UNIT_LIM_ALRT          (uint8_t) ID_MOTOR_UNIT_HIGH_NIBBLE | 0x08;
#define ID_MOTOR_UNIT_ENC_PPJR_SET      (uint8_t) ID_MOTOR_UNIT_HIGH_NIBBLE | 0x09;
#define ID_MOTOR_UNIT_MAX_JNT_REV_SET   (uint8_t) ID_MOTOR_UNIT_HIGH_NIBBLE | 0x0A;

// 