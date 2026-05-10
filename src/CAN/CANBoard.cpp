#include "CANBoard.h"

namespace can {

CANBoard::CANBoard(robot::types::boardid_t board_id, CANDevice_t device)
    : board_id(board_id), device(device) {
    if (device.motorDomain) {
        // Set default modes
        CANPacket_t p = CANMotorPacket_BLDC_SetInputMode(
            Constants::JETSON_DEVICE, device,
            static_cast<uint8_t>(can::motor::control_mode_t::velocity),
            static_cast<uint8_t>(can::motor::input_mode_t::vel_ramp)
        );
        sendCANPacket(p);

        // Ping motor for configs (max vel)
        this->vel_limit = 10;

        // Has watchdog?
    }

    if (device.peripheralDomain) {
        // Peripheral initialization?
    }
}

void CANBoard::setMotorPower(double power) {
    if (!this->device.motorDomain) {
        LOG_F(WARNING, "setMotorPower called for board not in motor domain!");
        return;
    }

    // Fetch motor states

    // Ensure motor control mode is velocity
    // Ensure motor state is closed loop control
    this->setMotorState(can::motor::axis_state_t::closed_loop_control);

    if (power == 0.0) {
        if (static_cast<uint8_t>(this->board_id) < 5) {   // hack for wheels + base
            this->setMotorState(can::motor::axis_state_t::idle);
        } else if (this->board_id == robot::types::boardid_t::shoulder || this->board_id == robot::types::boardid_t::elbow) {
            // Set motor general lockin vel to 0
            this->setMotorState(can::motor::axis_state_t::lockin_spin);
        }
    } else {
        // Mapping power to a target velocity
        int8_t input_vel = static_cast<int8_t>(power * this->vel_limit);
        // Make CANPacket_t
        CANPacket_t p = CANMotorPacket_BLDC_SetInputVelocity(
            Constants::JETSON_DEVICE, this->device, input_vel, 0.0f
        );

        // Send packet
        sendCANPacket(p);
    }
}

void CANBoard::setMotorState(can::motor::axis_state_t state) {
    if (!this->device.motorDomain) {
        LOG_F(WARNING, "setMotorState called for board not in motor domain!");
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
        LOG_F(WARNING, "setMotorPower called for board not in motor domain!");
        return;
    }

    // Make CANPacket_t
    CANPacket_t p = CANMotorPacket_BLDC_SetInputVelocity(
        Constants::JETSON_DEVICE, this->device, velocity, 0.0f
    );

    // Send packet
    sendCANPacket(p);
}

void CANBoard::read(uint16_t endpoint) {
    LOG_F(INFO, "read");
    CANPacket_t p = CANMotorPacket_BLDC_DirectRead(
        Constants::JETSON_DEVICE, this->device, endpoint
    );
    sendCANPacket(p);
}

} // namespace can