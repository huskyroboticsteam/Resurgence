#include "CANMotor.h"

namespace can {

CANBoard::CANBoard(robot::types::boardid_t board_id, CANDevice_t device)
    : board_id(board_id), device(device) {
    if (device.motorDomain) {
        // Set default modes
        CANPacket_t p = CANMotorPacket_BLDC_SetInputMode(
            Constants::JETSON_DEVICE, device,
            control_mode_t::velocity,
            input_mode_t::vel_ramp
        )
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
        LOG_F(WARNING, "setMotorPower called for board not in motor domain!")
        return;
    }

    // Fetch motor states

    // Mapping power to a target velocity
    int8_t input_vel = static_cast<int8_t>(power * this->vel_limit);

    // Ensure motor control mode is velocity
    // Ensure motor state is closed loop control

    // Make CANPacket_t
    CANPacket_t p = CANMotorPacket_BLDC_SetInputVelocity(
        Constants::JETSON_DEVICE, this->device, input_vel, 0.0f
    );

    // Send packet
    sendCANPacket(p);
}

} // namespace can