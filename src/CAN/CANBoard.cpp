#include "CANBoard.h"
#include "../world_interface/real_world_constants.h"

#include <nlohmann/json.hpp>

namespace can {

CANBoard::CANBoard(robot::types::boardid_t board_id, CANDevice_t device)
    : board_id(board_id), device(device) {
    if (device.motorDomain) {
        if (auto it = robot::boardInversionMap.find(board_id); it != robot::boardInversionMap.end()) {
            this->inversion_factor = it->second;
        }

        if (board_id == robot::types::boardid_t::hand) {
            // Skip ODrive config stuff
            return;
        }

        // Set default modes
        CANPacket_t p = CANMotorPacket_BLDC_SetInputMode(
            Constants::JETSON_DEVICE, device,
            static_cast<uint8_t>(can::motor::control_mode_t::velocity),
            static_cast<uint8_t>(can::motor::input_mode_t::vel_ramp)
        );
        sendCANPacket(p);

        // Ping motor for configs
        if (nlohmann::json endpoint = getEndpoint(this->board_id, "axis0.controller.config.vel_limit"); endpoint != nullptr) {
            endpointid_t endpoint_id = endpoint["id"];
            addDirectReadCallback(this->device, endpoint_id, [this, endpoint_id](auto p, std::unique_lock<std::shared_mutex> lock) {
                this->vel_limit = p.value_float;

                // We only need this once, remove after we get a response
                removeDirectReadCallback(this->device, endpoint_id);
            });

            this->read(endpoint_id);
        }

        if (nlohmann::json endpoint = getEndpoint(this->board_id, "axis0.config.enable_watchdog"); endpoint != nullptr) {
            endpointid_t endpoint_id = endpoint["id"];
            addDirectReadCallback(this->device, endpoint_id, [this, endpoint_id](auto p, std::unique_lock<std::shared_mutex> lock) {
                this->watchdog = p.value_bool;

                // We only need this once, remove after we get a response
                removeDirectReadCallback(this->device, endpoint_id);
            });

            this->read(endpoint_id);
        }
    }

    if (device.peripheralDomain) {
        // Peripheral initialization?
    }
}

void CANBoard::setMotorPower(double power) {
    if (!this->device.motorDomain) {
        LOG_F(WARNING, "setMotorPower called for %s board not in motor domain!", util::to_string(this->board_id).c_str());
        return;
    }

    if (power == 0.0) {
        if (this->board_id == robot::types::boardid_t::shoulder || this->board_id == robot::types::boardid_t::elbow) {
            // Set brake
            this->setBrake(BRAKE_ON);
        }

        this->setMotorState(can::motor::axis_state_t::idle);

        // Make CANPacket_t
        CANPacket_t p = CANMotorPacket_BLDC_SetInputVelocity(
            Constants::JETSON_DEVICE, this->device, 0.0f, 0.0f
        );

        // Send packet
        sendCANPacket(p);
    } else {
        // Ensure motor state is closed loop control
        this->setMotorState(can::motor::axis_state_t::closed_loop_control);
        // Mapping power to a target velocity
        this->input_vel = static_cast<float>(power * this->vel_limit) * this->inversion_factor;

        if (this->board_id == robot::types::boardid_t::shoulder || this->board_id == robot::types::boardid_t::elbow) {
            this->setBrake(BRAKE_OFF);
        }

        // Make CANPacket_t
        CANPacket_t p = CANMotorPacket_BLDC_SetInputVelocity(
            Constants::JETSON_DEVICE, this->device, input_vel, 0.0f
        );

        // Send packet
        sendCANPacket(p);

        // Double-check velocity set correctly
        if (nlohmann::json endpoint = getEndpoint(this->board_id, "axis0.controller.input_vel"); endpoint != nullptr) {
            endpointid_t endpoint_id = endpoint["id"];
            addDirectReadCallback(this->device, endpoint_id, [=](auto decoded, std::unique_lock<std::shared_mutex> lock) {
                if (decoded.value_float != input_vel) {
                    LOG_F(ERROR, "Expected %f, got %f", this->input_vel, decoded.value_float);
                }
            });

            this->read(endpoint_id);
        }
    }
}

void CANBoard::setMotorState(can::motor::axis_state_t state) {
    if (!this->device.motorDomain) {
        LOG_F(WARNING, "setMotorState called for %s board not in motor domain!", util::to_string(this->board_id).c_str());
        return;
    }

    uint32_t axis_state = static_cast<uint32_t>(state);
    CANPacket_t p = CANMotorPacket_BLDC_SetAxisState(
        Constants::JETSON_DEVICE, this->device, axis_state
    );
    sendCANPacket(p);
}

void CANBoard::setMotorVel(int8_t velocity) {
    if (!this->device.motorDomain) {
        LOG_F(WARNING, "setMotorVel called for %s board not in motor domain!", util::to_string(this->board_id).c_str());
        return;
    }

    float rot_vel = velocity / Constants::MILLIDEGREES_PER_REV;

    // Make CANPacket_t
    CANPacket_t p = CANMotorPacket_BLDC_SetInputVelocity(
        Constants::JETSON_DEVICE, this->device, rot_vel, 0.0f
    );

    // Send packet
    sendCANPacket(p);
}

void CANBoard::setStepperRevs(float revs) {
    if (!this->device.motorDomain) {
        LOG_F(WARNING, "setStepperRevs called for %s board not in motor domain!", util::to_string(this->board_id).c_str());
        return;
    }

    CANPacket_t p = CANMotorPacket_Stepper_DriveRevolutions(
        Constants::JETSON_DEVICE, this->device, revs
    );
    sendCANPacket(p);
}

void CANBoard::setActuator(int8_t out) {
    if (!this->device.peripheralDomain) {
        LOG_F(WARNING, "setActuator called for %s board not in peripheral domain!", util::to_string(this->board_id).c_str());
        return;
    }

    // TODO: hard-coded peripheral ID
    CANPacket_t p = CANPeripheralPacket_SetLinearActuator(
        Constants::JETSON_DEVICE, this->device, 2, out
    );
    sendCANPacket(p);
}

void CANBoard::setBrake(uint8_t state) {
    auto it = robot::boardBrakeIDMap.find(this->board_id);
    if (it == robot::boardBrakeIDMap.end()) {
        LOG_F(WARNING, "setBrake called for %s that does not have a brake!", util::to_string(this->board_id).c_str());
        return;
    }

    // TODO: hack, but we only have one braking board sooo    
    CANPacket_t p = CANPeripheralPacket_SetBrakes(
        Constants::JETSON_DEVICE, CANDevice_t{1, 0, 0, CAN_UUID_TELEMETRY}, it->second, state
    );
    sendCANPacket(p);
}

// TODO: hard coded
void CANBoard::setPWMDutyCycle(uint8_t peripheralID, float dutyCycle) {
    CANPacket_t p = CANPeripheralPacket_SetPWMDutyCycle(
        Constants::JETSON_DEVICE, CANDevice_t{1, 1, 0, CAN_UUID_HAND}, peripheralID, dutyCycle
    );
    sendCANPacket(p);
}

// TODO: hard coded
void CANBoard::setServoAngle(float angle) {
    CANPacket_t p = CANPeripheralPacket_SetServoAngle(
        Constants::JETSON_DEVICE, CANDevice_t{1, 0, 0, CAN_UUID_TELEMETRY}, 4, angle
    );
    sendCANPacket(p);
}

void CANBoard::read(endpointid_t endpoint) {
    CANPacket_t p = CANMotorPacket_BLDC_DirectRead(
        Constants::JETSON_DEVICE, this->device, endpoint
    );
    sendCANPacket(p);
}

} // namespace can