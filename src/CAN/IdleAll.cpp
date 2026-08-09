#include "CAN.h"
#include "CANBoard.h"
#include "../world_interface/real_world_constants.h"

int main() {
    can::initCAN();

    std::unordered_map<robot::types::boardid_t, std::shared_ptr<can::CANBoard>> board_ptrs;

    for (const auto& [board, device] : robot::boardDeviceMap) {
        if (!device.motorDomain) { continue; }
        std::shared_ptr<can::CANBoard> ptr = std::make_shared<can::CANBoard>(board, device);
        board_ptrs.insert({board, ptr});
    }

    while (true) {
        for (const auto& [board, ptr] : board_ptrs) {
            ptr->setMotorState(can::motor::axis_state_t::idle);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}