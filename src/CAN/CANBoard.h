#pragma once

#include "CAN.h"
#include "../world_interface/data.h"

#include <mutex>
#include <shared_mutex>

#define BRAKE_ON 0
#define BRAKE_OFF 1

namespace can {

class CANBoard {
  public:
    CANBoard(robot::types::boardid_t board_id, CANDevice_t device);

    // Motor
    void setMotorPower(double power);
    void setMotorState(can::motor::axis_state_t state);
    void setMotorVel(int8_t velocity);
    void setStepperRevs(float revs);

    // Peripheral
    void setBrake(uint8_t state);

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
      // int32_t mdeg = data.getDataOrElse(0);
      // if (mdeg != 0) {
      //   LOG_F(INFO, "0x%x @ %d mdeg", this->device.deviceUUID, mdeg);
      // }
      this->position_mdeg = data;
    }

  private:
    robot::types::boardid_t board_id;
    CANDevice_t device;
    int8_t inversion_factor;
    float input_vel;

    // Configs read on startup
    float vel_limit;
    bool watchdog;

    // Estimates received
    std::shared_mutex board_mutex;
    robot::types::DataPoint<int32_t> position_mdeg;
};

} // namespace can