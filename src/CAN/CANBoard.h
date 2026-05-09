#pragma once

#include "../world_interface/data.h"
#include "CAN.h"

namespace can {

class CANBoard {
  public:
    CANBoard(robot::types::boardid_t board_id, CANDevice_t device);

    void setMotorPower(double power);

  private:
    robot::types::boardid_t board_id;
    CANDevice_t device;
    uint8_t vel_limit;

    void 
};

} // namespace can

// Map <CANCommand_t, DirectRead UUID>