#pragma once

#include "../world_interface/data.h"
#include "CAN.h"

namespace can {

class CANBoard {
  public:
    CANBoard(robot::types::boardid_t board_id, CANDevice_t device);

    // Motor
    void setMotorPower(double power);
    void setMotorState(can::motor::axis_state_t state);
    void setMotorVel(int8_t velocity);

    // Universal
    void read(uint16_t endpoint);

  private:
    robot::types::boardid_t board_id;
    CANDevice_t device;
    uint8_t vel_limit;
};

} // namespace can

// Map <CANCommand_t, DirectRead UUID>