#include "CAN.h"
#include "CANMotor.h"
#include "../world_interface/real_world_constants.h"

#include <iostream>

extern "C" {
#include <CANPacket.h>
}

using namespace std::chrono_literals;

int prompt(std::string_view message) {
	std::string str;
	int val;
	bool valid_input = false;
	do {
		std::cout << message << " > ";
		std::getline(std::cin, str);
		try {
			val = std::stoi(str, nullptr, 0);
			valid_input = true;
		} catch (const std::invalid_argument&) {
			std::cerr << "Input must be a number (any base), try again" << std::endl;
		} catch (const std::out_of_range&) {
			std::cerr << "Input too big for int type" << std::endl;
		}
	} while (!valid_input);
	return val;
}

int main() {
    can::initCAN();
    CANPacket_t p;

    int mode = prompt("0-Motor, 1-Full");

    if (mode == 0) {
        // Set all axis states to calibration
        for (const auto& [motor, device] : robot::boardUUIDMap) {
            if (!device.motorDomain) continue;
            LOG_F(INFO, "Calibrating 0x%x", device.deviceUUID);
            p = CANMotorPacket_BLDC_SetAxisState(Constants::JETSON_DEVICE, device, BLDC_AXIS_MOTOR_CALIBRATION);
            can::sendCANPacket(p);
        }

        LOG_F(INFO, "Waiting for calibration...");
        std::this_thread::sleep_for(8s);
        LOG_F(INFO, "Setting up closed loop control...");

        // Enable vel closed loop
        can::motor::setMotorMode(CANDevice_t{0, 1, 0, CAN_UUID_BLDC_FOREARM}, can::motor::motormode_t::vel);
        can::motor::setMotorMode(CANDevice_t{0, 1, 0, CAN_UUID_BLDC_WRIST_LEFT}, can::motor::motormode_t::vel);
        can::motor::setMotorMode(CANDevice_t{0, 1, 0, CAN_UUID_BLDC_WRIST_RIGHT}, can::motor::motormode_t::vel);

        p = CANMotorPacket_BLDC_SetAxisState(Constants::JETSON_DEVICE, CANDevice_t{0, 1, 0, CAN_UUID_BLDC_FOREARM}, BLDC_AXIS_CLOSED_LOOP_CONTROL);
        can::sendCANPacket(p);

        p = CANMotorPacket_BLDC_SetAxisState(Constants::JETSON_DEVICE, CANDevice_t{0, 1, 0, CAN_UUID_BLDC_WRIST_LEFT}, BLDC_AXIS_CLOSED_LOOP_CONTROL);
        can::sendCANPacket(p);

        p = CANMotorPacket_BLDC_SetAxisState(Constants::JETSON_DEVICE, CANDevice_t{0, 1, 0, CAN_UUID_BLDC_WRIST_RIGHT}, BLDC_AXIS_CLOSED_LOOP_CONTROL);
        can::sendCANPacket(p);

        LOG_F(INFO, "Finished!");
    } else if (mode == 1) {
        int uuid = static_cast<uint16_t>(prompt("Enter device uuid"));

        CANDevice_t device;
        device.deviceUUID = uuid;

        p = CANMotorPacket_BLDC_SetAxisState(Constants::JETSON_DEVICE, device, BLDC_AXIS_FULL_CALIBRATION_SEQUENCE);
        can::sendCANPacket(p);
    }
}