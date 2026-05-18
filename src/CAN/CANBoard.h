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

    robot::types::boardid_t getBoardID() const { return board_id; }
    CANDevice_t getDevice() const { return device; }
    robot::types::DataPoint<int32_t> getPosition() const {
      std::shared_lock lock(board_mutex);
      return this->position_mdeg;
    }

    void storePosition(robot::types::DataPoint<int32_t> data) const {
      std::lock_guard lock(board_mutex);
      this->position_mdeg = data;
    }

  private:
    robot::types::boardid_t board_id;
    CANDevice_t device;
    int8_t inversion_factor;

    // Configs read on startup
    float vel_limit;
    bool watchdog;

    // Estimates received
    std::mutex board_mutex;
    robot::types::DataPoint<int32_t> position_mdeg;

    // debug
    bool correct = false;
};

} // namespace can