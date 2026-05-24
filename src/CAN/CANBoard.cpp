#include "CAN.h"
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
            uint16_t endpoint_id = endpoint["id"];
            addDirectReadCallback(this->device, endpoint_id, [this, endpoint_id](auto p) {
                std::unique_lock lock(this->board_mutex);
                this->vel_limit = p.value_float;

                // We only need this once, remove after we get a response
                removeDirectReadCallback(this->device, endpoint_id);
            });

            // this->vel_limit = 0;
            this->read(endpoint_id);
        }

        if (nlohmann::json endpoint = getEndpoint(this->board_id, "axis0.config.enable_watchdog"); endpoint != nullptr) {
            uint16_t endpoint_id = endpoint["id"];
            addDirectReadCallback(this->device, endpoint_id, [this, endpoint_id](auto p) {
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
        LOG_F(WARNING, "setMotorPower called for board 0x%x not in motor domain!", this->device.deviceUUID);
        return;
    }

    // Fetch motor states

    // Ensure motor control mode is velocity
    // Ensure motor state is closed loop control
    this->setMotorState(can::motor::axis_state_t::closed_loop_control);

    if (power == 0.0) {
        if (this->watchdog) {
            this->setMotorState(can::motor::axis_state_t::idle);

            if (nlohmann::json endpoint = getEndpoint(this->board_id, "axis0.current_state"); endpoint != nullptr) {
                uint16_t endpoint_id = endpoint["id"];
                addDirectReadCallback(this->device, endpoint_id, [this, endpoint_id](auto decoded) {
                    if (decoded.value_uint8 != static_cast<uint8_t>(can::motor::axis_state_t::idle)) {
                        LOG_F(ERROR, "0x%x DID NOT LISTEN AND IS NOT IDLE AND IS INSTEAD %d", this->device.deviceUUID, decoded.value_uint8);
                        this->setMotorState(can::motor::axis_state_t::idle);
                        this->read(endpoint_id);
                    } else {
                        removeDirectReadCallback(this->device, endpoint_id);
                    }
                });

                this->read(endpoint_id);
            }
        } else if (this->board_id == robot::types::boardid_t::shoulder || this->board_id == robot::types::boardid_t::elbow) {
            // Set motor general lockin vel to 0
            this->setMotorState(can::motor::axis_state_t::lockin_spin);
        }

        if (nlohmann::json endpoint = getEndpoint(this->board_id, "axis0.controller.input_vel"); endpoint != nullptr) {
            uint16_t endpoint_id = endpoint["id"];
            removeDirectReadCallback(this->device, endpoint_id);
        }
    } else {
        // Mapping power to a target velocity
        float input_vel = static_cast<float>(power * this->vel_limit);
        input_vel *= this->inversion_factor;
        // Make CANPacket_t
        CANPacket_t p = CANMotorPacket_BLDC_SetInputVelocity(
            Constants::JETSON_DEVICE, this->device, input_vel, 0.0f
        );

        // Send packet
        sendCANPacket(p);

        if (nlohmann::json endpoint = getEndpoint(this->board_id, "axis0.controller.input_vel"); endpoint != nullptr) {
            uint16_t endpoint_id = endpoint["id"];
            addDirectReadCallback(this->device, endpoint_id, [input_vel, p, this, endpoint_id](auto decoded) {
                if (decoded.value_float != input_vel) {
                    LOG_F(ERROR, "Expected %f, got %f", input_vel, decoded.value_float);
                    sendCANPacket(p);
                }
            });

            this->read(endpoint_id);
        }
    }
}

void CANBoard::setMotorState(can::motor::axis_state_t state) {
    if (!this->device.motorDomain) {
        LOG_F(WARNING, "setMotorState called for board 0x%x not in motor domain!", this->device.deviceUUID);
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
        LOG_F(WARNING, "setMotorVel called for board 0x%x not in motor domain!", this->device.deviceUUID);
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
        LOG_F(WARNING, "setStepperRevs called for board 0x%x not in motor domain!", this->device.deviceUUID);
        return;
    }

    CANPacket_t p = CANMotorPacket_Stepper_DriveRevolutions(
        Constants::JETSON_DEVICE, this->device, revs
    );
    sendCANPacket(p);
}

void CANBoard::read(uint16_t endpoint) {
    CANPacket_t p = CANMotorPacket_BLDC_DirectRead(
        Constants::JETSON_DEVICE, this->device, endpoint
    );
    sendCANPacket(p);
}

} // namespace can