#include "../control/JacobianVelController.h"
#include "../navtypes.h"
#include "../utils/core.h"
#include "../utils/scheduler.h"
#include "../world_interface/data.h"
#include "CAN.h"
#include "CANMotor.h"
#include "CANUtils.h"

#include <algorithm>
#include <chrono>
#include <exception>
#include <iostream>
#include <mutex>
#include <thread>
#include <unordered_set>
#include <vector>

extern "C" {
#include <CANCommandIDs.h>
#include <CANPacket.h>

#include <HindsightCAN/CANPower.h>
#include <HindsightCAN/CANScience.h>
#include <Packets/Universal.h>
}

using namespace std::chrono_literals;

using can::motor::motormode_t;
using can::motor::motorstate_t;
using namespace robot::types;

enum class TestMode {
	ModeSet,
	State,
	Vel,
	RawCAN,
	NUM_MODES
};

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

	std::stringstream ss("What are you testing?\n");
	ss << static_cast<int>(TestMode::ModeSet) << " for MODE SET\n";
	ss << static_cast<int>(TestMode::State) << " for SET STATE\n";
	ss << static_cast<int>(TestMode::Vel) << " for VELOCITY CONTROL\n";
	ss << static_cast<int>(TestMode::RawCAN) << " for CAN\n";

	while (true) {
		int test_type = prompt(ss.str().c_str());

		if (test_type >= static_cast<int>(TestMode::NUM_MODES)) {
			std::cout << "Unrecognized response: " << test_type << std::endl;
			std::exit(1);
		}

		TestMode testMode = static_cast<TestMode>(test_type);

		while (true) {
			if (testMode == TestMode::ModeSet) {
				int uuid = static_cast<uint16_t>(prompt("Enter device uuid"));
				int mode = prompt("Enter mode (0 for PWM, 1 for PID)");

				CANDevice_t device;
				device.deviceUUID = uuid;
				std::cout << "got " << device.deviceUUID << std::endl;
				can::motor::setMotorMode(device, mode == 0 ? motormode_t::vel : motormode_t::pos);
			} else if (testMode == TestMode::State) {
				int uuid = static_cast<uint16_t>(prompt("Enter device uuid"));
				CANDevice_t device;
				device.deviceUUID = uuid;

				int state = prompt("Enter desired motor state");
				can::motor::motorstate_t motorState = static_cast<motorstate_t>(state);
				can::motor::setMotorState(device, motorState);
			} else if (testMode == TestMode::Vel) {
				int uuid = static_cast<uint16_t>(prompt("Enter device uuid"));
				CANDevice_t device;
				device.deviceUUID = uuid;

				can::motor::setMotorMode(device, motormode_t::vel);
				can::motor::setMotorState(device, motorstate_t::control);

				double vel = static_cast<double>(prompt("Enter velocity"));
				can::motor::setMotorPower(device, vel);

			// } else if (testMode == TestMode::Telemetry) {
			// 	if (!mode_has_been_set) {
			// 		CANDeviceUUID_t uuid = static_cast<uint16_t>(prompt("Enter device uuid"));
			// 		auto telemType = static_cast<can::telemtype_t>(prompt("Enter telemetry type"));
			// 		can::addDeviceTelemetryCallback(
			// 			uuid, telemType,
			// 			[](can::uuid_t uuid, can::telemtype_t telemType,
			// 			DataPoint<can::telemetry_t> data) {
			// 				std::cout << "Telemetry: uuid=" << static_cast<int>(uuid)
			// 						<< ", type=" << static_cast<int>(telemType)
			// 						<< static_cast<int>(telemType) << ", data=" << std::dec
			// 						<< data.getDataOrElse(0) << std::endl;
			// 			});
			// 		int telemPeriod = prompt("Telemetry timing (ms)");
			// 		bool useTimingPacket =
			// 			static_cast<bool>(prompt("What telemetry method?\n0 for pull packets\n1 "
			// 									"for telemetry timing packet"));
			// 		if (useTimingPacket) {
			// 			/* TO DO: Telemetry Packets

			// 			CANPacket packet;
			// 			AssembleTelemetryTimingPacket(
			// 				&packet, static_cast<uint8_t>(deviceID.first), deviceID.second,
			// 				static_cast<uint8_t>(telemType), telemPeriod);
			// 			can::sendCANPacket(packet);
			// 			*/
			// 		} else {
			// 			can::scheduleTelemetryPull(uuid, telemType,
			// 									std::chrono::milliseconds(telemPeriod));
			// 		}
			// 		mode_has_been_set = true;
			// 	}
			// 	std::this_thread::sleep_for(1s);
			} else if (testMode == TestMode::RawCAN) {
				uint8_t pr = prompt("priority");
				uint8_t uuid = prompt("uuid");
				uint8_t command = prompt("command");
				uint8_t dlc = prompt("add'l. data bits");
				uint8_t data[dlc + 1];
				data[0] = command;

				for (int i = 1; i <= dlc; i++) {
					data[i] = prompt("bit");
				}

				// manual construction of a generic packet
				CANPacket_t p = {};
				p.device.deviceUUID = uuid;
				p.priority = static_cast<CANPriority_t>(pr);
				p.command = command;
				p.senderUUID = CAN_UUID_JETSON;
				p.contentsLength = dlc;
				for (int i = 0; i < p.contentsLength && i < 6; i++) {
					p.contents[i] = data[i + 1];
				}

				can::sendCANPacket(p);
				can::printCANPacket(p);
			}
		}
	}
}