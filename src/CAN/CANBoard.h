#pragma once

#include "CAN.h"
#include "../world_interface/data.h"

#include <mutex>
#include <shared_mutex>

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

    robot::types::DataPoint<int32_t> getPosition() {
      std::shared_lock lock(board_mutex);
      return this->position_mdeg;
    }

    void storePosition(const robot::types::DataPoint<int32_t> data) {
      std::unique_lock lock(board_mutex);
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
    std::shared_mutex board_mutex;
    robot::types::DataPoint<int32_t> position_mdeg;
};

} // namespace can