#include "CANMotor.h"

#include "CAN.h"
#include "CANUtils.h"

#include <chrono>
#include <cmath>
#include <thread>

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
    // Emergency stop for AVR/PSoC motors
    AssembleGroupBroadcastingEmergencyStopPacket(&p, static_cast<uint8_t>(devicegroup_t::motor),
                                                 ESTOP_ERR_GENERAL);
    sendCANPacket(p);
    // Emergency stop for ODrive motors
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
    // Set axis state to idle initially
    AssembleODriveSetAxisStatePacket(&p, node_id, 1); // AXIS_STATE_IDLE
    sendCANPacket(p);
    std::this_thread::sleep_for(1000us);
    // Clear any errors
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
            control_mode = 3; // CONTROL_MODE_POSITION_CONTROL
            input_mode = 1;   // INPUT_MODE_PASSTHROUGH
            break;
        case odrive_motormode_t::velocity:
            control_mode = 2; // CONTROL_MODE_VELOCITY_CONTROL
            input_mode = 1;   // INPUT_MODE_PASSTHROUGH
            break;
        case odrive_motormode_t::torque:
            control_mode = 1; // CONTROL_MODE_TORQUE_CONTROL
            input_mode = 1;   // INPUT_MODE_PASSTHROUGH
            break;
        default:
            return;
    }
    AssembleODriveSetControllerModePacket(&p, node_id, control_mode, input_mode);
    sendCANPacket(p);
    std::this_thread::sleep_for(1000us);
    // Set axis state to closed-loop control to activate the mode
    AssembleODriveSetAxisStatePacket(&p, node_id, 8); // AXIS_STATE_CLOSED_LOOP_CONTROL
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
    AssembleODriveSetInputPosPacket(&p, node_id, position, vel_ff, torque_ff);
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
    AssemblePCAServoPacket(&p, static_cast<uint8_t>(devicegroup_t::motor), serial, servoNum,
                           angle);
    sendCANPacket(p);
}

DataPoint<int32_t> getMotorPosition(deviceserial_t serial) {
    return getDeviceTelemetry(std::make_pair(devicegroup_t::motor, serial),
                              telemtype_t::angle);
}

DataPoint<float> getODrivePosition(uint8_t node_id) {
    auto data = getDeviceTelemetry(std::make_pair(devicegroup_t::motor, node_id),
                                   telemtype_t::angle);
    if (!data) return {};
    float position = static_cast<float>(data.getData());
    return DataPoint<float>(data.getTime(), position);
}

DataPoint<float> getODriveVelocity(uint8_t node_id) {
    auto data = getDeviceTelemetry(std::make_pair(devicegroup_t::motor, node_id),
                                   telemtype_t::gyro_x); // Using gyro_x for velocity
    if (!data) return {};
    float velocity = static_cast<float>(data.getData());
    return DataPoint<float>(data.getTime(), velocity);
}

DataPoint<std::pair<float, float>> getODriveTemperature(uint8_t node_id) {
    auto data = getDeviceTelemetry(std::make_pair(devicegroup_t::motor, node_id),
                                   telemtype_t::temp);
    if (!data) return {};
    float temp = static_cast<float>(data.getData());
    // Since temp contains both FET and motor temperatures packed, we'll assume it's split
    float fet_temp = temp; // Simplified; in reality, need to decode both
    float motor_temp = temp;
    return DataPoint<std::pair<float, float>>(data.getTime(), {fet_temp, motor_temp});
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
    const std::function<void(deviceserial_t serial,
                             DataPoint<LimitSwitchData> limitSwitchData)>& callback) {
    auto id = std::make_pair(devicegroup_t::motor, serial);
    auto func = [callback](deviceid_t deviceID, telemtype_t,
                           DataPoint<telemetry_t> telemData) {
        if (telemData) {
            LimitSwitchData data = telemData.getData();
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
