#include "CAN.h"
#include "CANBoard.h"

#include <algorithm>
#include <iostream>

enum class TestMode {
	State,
	Power,
	Read,
	Stepper,
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

bool correct = false;

int main() {
	can::initCAN();

	std::stringstream ss("What are you testing?\n");
	ss << static_cast<int>(TestMode::State) << " for SET STATE\n";
	ss << static_cast<int>(TestMode::Power) << " for POWER CONTROL\n";
	ss << static_cast<int>(TestMode::Read) << " for DIRECT READ\n";
	ss << static_cast<int>(TestMode::Stepper) << " for STEPPER\n";
	ss << static_cast<int>(TestMode::RawCAN) << " for RAW CAN\n";

	while (true) {
		int test_type = prompt(ss.str().c_str());

		if (test_type >= static_cast<int>(TestMode::NUM_MODES)) {
			std::cout << "Unrecognized response: " << test_type << std::endl;
			std::exit(1);
		}

		TestMode testMode = static_cast<TestMode>(test_type);
		uint16_t uuid = static_cast<uint16_t>(prompt("Enter device uuid"));
		// TODO: Assuming motor domain for now
		CANDevice_t device = CANDevice_t{0, 1, 0, uuid};

		std::shared_ptr<can::CANBoard> board = std::make_shared<can::CANBoard>(robot::types::boardid_t::debug1, device);

		if (testMode == TestMode::Read && static_cast<uint8_t>(prompt("0 for S1, 1 for Pro")) == 1) {
			board = std::make_shared<can::CANBoard>(robot::types::boardid_t::debug2, device);
		}

		while (true) {
			if (testMode == TestMode::State) {
				std::stringstream state_msg("");
				state_msg << "Enter desired motor state:\n";
				state_msg << static_cast<int>(can::motor::axis_state_t::idle) << " idle\n";
				state_msg << static_cast<int>(can::motor::axis_state_t::closed_loop_control) << " closed loop control\n";

				int state = prompt(state_msg.str().c_str());
				can::motor::axis_state_t motor_state = static_cast<can::motor::axis_state_t>(state);
				board->setMotorState(motor_state);

				correct = false;
				if (nlohmann::json endpoint = can::getEndpoint(board->getBoardID(), "axis0.current_state"); endpoint != nullptr) {
					uint16_t endpoint_id = endpoint["id"];
					can::addDirectReadCallback(board->getDevice(), endpoint_id, [board, motor_state, endpoint_id](auto decoded) {
						if (decoded.value_uint8 != static_cast<uint8_t>(motor_state)) {
							LOG_F(ERROR, "0x%x DID NOT LISTEN AND IS NOT STATE %d AND IS INSTEAD %d", board->getDevice().deviceUUID, static_cast<uint8_t>(motor_state), decoded.value_uint8);
							board->setMotorState(motor_state);
							board->read(endpoint_id);
						} else {
							can::removeDirectReadCallback(board->getDevice(), endpoint_id);
							correct = true;
						}
					});

					board->read(endpoint_id);
				}

				while (!correct);
			} else if (testMode == TestMode::Power) {
				std::string input;
				std::cout << "Enter power [-1.0, 1.0]: ";
				std::getline(std::cin, input);
				float power = std::stof(input);

				board->setMotorPower(power);
			} else if (testMode == TestMode::Read) {
				std::string input;
				std::cout << "Enter endpoint name" << " > ";
				std::getline(std::cin, input);

				if (nlohmann::json endpoint = can::getEndpoint(board->getBoardID(), input); endpoint != nullptr) {
					uint16_t endpoint_id = endpoint["id"];
					// std::cout << "endpoint " << input << " has id=" << endpoint_id << std::endl;
					can::addDirectReadCallback(board->getDevice(), endpoint_id, [board, input, endpoint](auto decoded) {
						std::stringstream rs("");
						rs << input << " from 0x" << std::hex << board->getDevice().deviceUUID << " [";

						std::string type = endpoint["type"];
						rs << type << "]: ";
						if (type == "uint32") {
							rs << decoded.value_uint32;
						} else if (type == "int32") {
							rs << decoded.value_int32;
						} else if (type == "uint16") {
							rs << decoded.value_uint16;
						} else if (type == "uint8") {
							rs << decoded.value_uint8;
						} else if (type == "float") {
							rs << decoded.value_float;
						} else if (type == "bool") {
							rs << (decoded.value_bool ? "true" : "false");
						}

						std::cout << rs.str().c_str() << std::endl;

						can::removeDirectReadCallback(board->getDevice(), endpoint["id"]);
					});

					board->read(endpoint_id);
					std::this_thread::sleep_for(std::chrono::milliseconds(500));
				} else {
					std::cout << "Unknown endpoint" << std::endl;
					continue;
				}
			} else if (testMode == TestMode::Stepper) {
				std::string input;
				std::cout << "Enter revs: ";
				std::getline(std::cin, input);
				float revs = std::stof(input);
				CANPacket_t packet = CANMotorPacket_Stepper_DriveRevolutions(Constants::JETSON_DEVICE, device, revs);
				can::printCANPacket(packet);
				can::sendCANPacket(packet);
			} else if (testMode == TestMode::RawCAN) {
				uint8_t periphID = prompt("peripheral ID");
				std::string input;
				std::cout << "Enter pwm duty cycle: ";
				std::getline(std::cin, input);
				float dutyCycle = std::stof(input);
				CANPacket_t packet = CANPeripheralPacket_SetPWMDutyCycle(Constants::JETSON_DEVICE, device, periphID, dutyCycle);
				packet.command = CAN_ACK(packet.command);
				can::printCANPacket(packet);
				can::sendCANPacket(packet);
				// uint8_t pr = prompt("priority");
				// uint8_t command = prompt("command");
				// uint8_t dlc = prompt("add'l. data bits");
				// if (dlc > 5) {
				// 	std::cout << "Too many data bits" << std::endl;
				// 	continue;
				// }
				// uint8_t data[dlc + 1];
				// data[0] = command;

				// for (int i = 1; i <= dlc; i++) {
				// 	data[i] = prompt("bit " + std::to_string(i));
				// }

				// // // manual construction of a generic packet
				// CANPacket_t p = {};
				// p.device = device;
				// p.priority = static_cast<CANPriority_t>(pr);
				// p.command = command;
				// p.senderUUID = CAN_UUID_JETSON;
				// p.contentsLength = dlc;
				// for (int i = 0; i < p.contentsLength && i < 6; i++) {
				// 	p.contents[i] = data[i + 1];
				// }

				// can::printCANPacket(p);
				// can::sendCANPacket(p);
			}
		}
	}
}