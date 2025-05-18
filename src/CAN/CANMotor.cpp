#include "CANMotor.h"
#include "CAN.h"
#include "CANUtils.h"

#include <chrono>
#include <cmath>
#include <thread>
#include <any>

extern "C" {
#include <HindsightCAN/CANCommon.h>
#include <HindsightCAN/CANMotorUnit.h>
#include <HindsightCAN/CANPacket.h>
}

using namespace std::chrono_literals;
using robot::types::DataPoint;
using robot::types::LimitSwitchData;

namespace can::motor {

void initEncoder(deviceserial_t serial, bool invertEncoder, bool zeroEncoder,
                 int32_t pulsesPerJointRev,
                 std::optional<std::chrono::milliseconds> telemetryPeriod) {
    auto motorGroupCode = static_cast<uint8_t>(devicegroup_t::motor);
    CANPacket p;
    AssembleEncoderInitializePacket(&p, motorGroupCode, serial, sensor_t::encoder,
                                    invertEncoder, zeroEncoder);
    sendCANPacket(p);
    std::this_thread::sleep_for(1000us);
    AssembleEncoderPPJRSetPacket(&p, motorGroupCode, serial, pulsesPerJointRev);
    sendCANPacket(p);
    std::this_thread::sleep_for(1000us);
    if (telemetryPeriod) {
        scheduleTelemetryPull(std::make_pair(devicegroup_t::motor, serial), telemtype_t::angle,
                              telemetryPeriod.value());
    }
}

void initPotentiometer(deviceserial_t serial, int32_t posLo, int32_t posHi, uint16_t adcLo,
                       uint16_t adcHi,
                       std::optional<std::chrono::milliseconds> telemetryPeriod) {
    CANPacket packet;
    auto group = static_cast<uint8_t>(devicegroup_t::motor);
    AssemblePotHiSetPacket(&packet, group, serial, adcHi, posHi);
    sendCANPacket(packet);
    std::this_thread::sleep_for(1ms);
    AssemblePotLoSetPacket(&packet, group, serial, adcLo, posLo);
    sendCANPacket(packet);
    if (telemetryPeriod) {
        scheduleTelemetryPull(std::make_pair(devicegroup_t::motor, serial), telemtype_t::angle,
                              telemetryPeriod.value());
    }
}

void emergencyStopMotors() {
    CANPacket p;
    AssembleGroupBroadcastingEmergencyStopPacket(&p, static_cast<uint8_t>(devicegroup_t::motor),
                                                 ESTOP_ERR_GENERAL);
    sendCANPacket(p);
    AssembleODriveEstopPacket(&p, NODE_ID_1);
    sendCANPacket(p);
    AssembleODriveEstopPacket(&p, NODE_ID_2);
    sendCANPacket(p);
    std::this_thread::sleep_for(1000us);
}

void initMotor(deviceserial_t serial) {
    setMotorMode(serial, motormode_t::pwm);
    std::this_thread::sleep_for(1000us);
}

void initODriveMotor(uint8_t node_id) {
    CANPacket p;
    AssembleODriveSetAxisStatePacket(&p, node_id, 1);
    sendCANPacket(p);
    std::this_thread::sleep_for(1000us);
    AssembleODriveClearErrorsPacket(&p, node_id, 0);
    sendCANPacket(p);
    std::this_thread::sleep_for(1000us);
}

void setLimitSwitchLimits(deviceserial_t serial, int32_t lo, int32_t hi) {
    CANPacket p;
    auto motorGroupCode = static_cast<uint8_t>(devicegroup_t::motor);
    AssembleLimSwEncoderBoundPacket(&p, motorGroupCode, serial, 0, lo);
    sendCANPacket(p);
    std::this_thread::sleep_for(1ms);
    AssembleLimSwEncoderBoundPacket(&p, motorGroupCode, serial, 1, hi);
    sendCANPacket(p);
}

void setMotorPIDConstants(deviceserial_t serial, int32_t kP, int32_t kI, int32_t kD) {
    CANPacket p;
    auto motorGroupCode = static_cast<uint8_t>(devicegroup_t::motor);
    AssemblePSetPacket(&p, motorGroupCode, serial, kP);
    sendCANPacket(p);
    std::this_thread::sleep_for(1ms);
    AssembleISetPacket(&p, motorGroupCode, serial, kI);
    sendCANPacket(p);
    std::this_thread::sleep_for(1ms);
    AssembleDSetPacket(&p, motorGroupCode, serial, kD);
    sendCANPacket(p);
    std::this_thread::sleep_for(1ms);
}

void setODrivePIDGains(uint8_t node_id, float pos_gain, float vel_gain, float vel_integrator_gain) {
    CANPacket p;
    AssembleODriveSetPosGainPacket(&p, node_id, pos_gain);
    sendCANPacket(p);
    std::this_thread::sleep_for(1ms);
    AssembleODriveSetVelGainsPacket(&p, node_id, vel_gain, vel_integrator_gain);
    sendCANPacket(p);
    std::this_thread::sleep_for(1ms);
}

void setMotorMode(deviceserial_t serial, motormode_t mode) {
    CANPacket p;
    AssembleModeSetPacket(&p, static_cast<uint8_t>(devicegroup_t::motor), serial,
                          static_cast<uint8_t>(mode));
    sendCANPacket(p);
    std::this_thread::sleep_for(1000us);
}

void setODriveMode(uint8_t node_id, odrive_motormode_t mode) {
    CANPacket p;
    uint32_t control_mode, input_mode;
    switch (mode) {
        case odrive_motormode_t::position:
            control_mode = 3;
            input_mode = 1;
            break;
        case odrive_motormode_t::velocity:
            control_mode = 2;
            input_mode = 1;
            break;
        case odrive_motormode_t::torque:
            control_mode = 1;
            input_mode = 1;
            break;
        default:
            return;
    }
    AssembleODriveSetControllerModePacket(&p, node_id, control_mode, input_mode);
    sendCANPacket(p);
    std::this_thread::sleep_for(1000us);
    AssembleODriveSetAxisStatePacket(&p, node_id, 8);
    sendCANPacket(p);
    std::this_thread::sleep_for(1000us);
}

void setMotorPower(deviceserial_t serial, double power) {
    power = std::min(std::max(power, -1.0), 1.0);
    int powerInt = std::round(power * std::numeric_limits<int16_t>::max());
    int16_t dutyCycle = static_cast<int16_t>(powerInt);
    setMotorPower(serial, dutyCycle);
}

void setMotorPower(deviceserial_t serial, int16_t power) {
    CANPacket p;
    AssemblePWMDirSetPacket(&p, static_cast<uint8_t>(devicegroup_t::motor), serial, power);
    sendCANPacket(p);
}

void setMotorPIDTarget(deviceserial_t serial, int32_t target) {
    CANPacket p;
    AssemblePIDTargetSetPacket(&p, static_cast<uint8_t>(devicegroup_t::motor), serial, target);
    sendCANPacket(p);
}

void setODrivePosition(uint8_t node_id, float position, float vel_ff, float torque_ff) {
    CANPacket p;
    p.id = ((node_id & 0x1F) << 6) | ID_ODRIVE_SET_INPUT_POS;
    p.dlc = 8;
    p.rtr = 0;
    union { float f; uint32_t u; } float_to_uint;
    float_to_uint.f = position;
    p.data[0] = float_to_uint.u & 0xFF;
    p.data[1] = (float_to_uint.u >> 8) & 0xFF;
    p.data[2] = (float_to_uint.u >> 16) & 0xFF;
    p.data[3] = (float_to_uint.u >> 24) & 0xFF;
    int16_t vel_ff_scaled = static_cast<int16_t>(vel_ff / 0.0005);
    int16_t torque_ff_scaled = static_cast<int16_t>(torque_ff / 0.001);
    p.data[4] = vel_ff_scaled & 0xFF;
    p.data[5] = (vel_ff_scaled >> 8) & 0xFF;
    p.data[6] = torque_ff_scaled & 0xFF;
    p.data[7] = (torque_ff_scaled >> 8) & 0xFF;
    sendCANPacket(p);
}

void setODriveVelocity(uint8_t node_id, float velocity, float torque_ff) {
    CANPacket p;
    AssembleODriveSetInputVelPacket(&p, node_id, velocity, torque_ff);
    sendCANPacket(p);
}

void setODriveTorque(uint8_t node_id, float torque) {
    CANPacket p;
    AssembleODriveSetInputTorquePacket(&p, node_id, torque);
    sendCANPacket(p);
}

void setServoPos(deviceserial_t serial, uint8_t servoNum, int32_t angle) {
    CANPacket p;
    AssemblePCAServoPacket(&p, static_cast<uint8_t>(devicegroup_t::motor), serial, servoNum, angle);
    sendCANPacket(p);
}

DataPoint<int32_t> getMotorPosition(deviceserial_t serial) {
    auto data = getDeviceTelemetry(std::make_pair(devicegroup_t::motor, serial), telemtype_t::angle);
    if (!data) return {};
    return DataPoint<int32_t>(data.getTime(), std::any_cast<int32_t>(data.getData()));
}

DataPoint<float> getODrivePosition(uint8_t node_id) {
    auto data = getDeviceTelemetry(std::make_pair(devicegroup_t::motor, node_id), telemtype_t::angle);
    if (!data) return {};
    return DataPoint<float>(data.getTime(), std::any_cast<float>(data.getData()));
}

DataPoint<float> getODriveVelocity(uint8_t node_id) {
    auto data = getDeviceTelemetry(std::make_pair(devicegroup_t::motor, node_id), telemtype_t::velocity);
    if (!data) return {};
    return DataPoint<float>(data.getTime(), std::any_cast<float>(data.getData()));
}

DataPoint<std::pair<float, float>> getODriveTemperature(uint8_t node_id) {
    auto data = getDeviceTelemetry(std::make_pair(devicegroup_t::motor, node_id), telemtype_t::odrive_temp);
    if (!data) return {};
    return DataPoint<std::pair<float, float>>(data.getTime(), std::any_cast<std::pair<float, float>>(data.getData()));
}

void pullMotorPosition(deviceserial_t serial) {
    pullDeviceTelemetry(std::make_pair(devicegroup_t::motor, serial), telemtype_t::angle);
}

void pullODriveEncoderEstimates(uint8_t node_id) {
    SendODriveRTRRequest(node_id, ID_ODRIVE_GET_ENCODER_ESTIMATES);
}

void pullODriveTemperature(uint8_t node_id) {
    SendODriveRTRRequest(node_id, ID_ODRIVE_GET_TEMPERATURE);
}

callbackid_t addLimitSwitchCallback(
    deviceserial_t serial,
    const std::function<void(deviceserial_t serial, DataPoint<LimitSwitchData> limitSwitchData)>& callback) {
    auto id = std::make_pair(devicegroup_t::motor, serial);
    auto func = [callback](deviceid_t deviceID, telemtype_t,
                           DataPoint<std::any> telemData) {
        if (telemData) {
            LimitSwitchData data = std::any_cast<LimitSwitchData>(telemData.getData());
            callback(deviceID.second, {telemData.getTime(), data});
        } else {
            callback(deviceID.second, {});
        }
    };
    return addDeviceTelemetryCallback(id, telemtype_t::limit_switch, func);
}

void removeLimitSwitchCallback(callbackid_t id) {
    removeDeviceTelemetryCallback(id);
}
} // namespace can::motor
