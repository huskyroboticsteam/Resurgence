#include "CAN.h"
#include "CANBoard.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <shared_mutex>

enum class TestMode {
	State,
	Power,
	Read,
	Write,
	Stepper,
	Peripheral,
	RawCAN,
	Debug,
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

	uint16_t uuid = static_cast<uint16_t>(prompt("Enter device uuid"));
	CANDevice_t device = CANDevice_t{1, 1, 1, uuid};

	std::shared_ptr<can::CANBoard> board = std::make_shared<can::CANBoard>(robot::types::boardid_t::debug1, device);

	std::stringstream ss("What are you testing?\n");
	ss << static_cast<int>(TestMode::State) << " for SET STATE\n";
	ss << static_cast<int>(TestMode::Power) << " for POWER CONTROL\n";
	ss << static_cast<int>(TestMode::Read) << " for DIRECT READ\n";
	ss << static_cast<int>(TestMode::Write) << " for DIRECT WRITE\n";
	ss << static_cast<int>(TestMode::Stepper) << " for STEPPER\n";
	ss << static_cast<int>(TestMode::Peripheral) << " for PERIPHERAL\n";
	ss << static_cast<int>(TestMode::RawCAN) << " for RAW CAN\n";

	int brake = 0;

	while (true) {
		int test_type = prompt(ss.str().c_str());

		if (test_type >= static_cast<int>(TestMode::NUM_MODES)) {
			std::cout << "Unrecognized response: " << test_type << std::endl;
			std::exit(1);
		}

		TestMode testMode = static_cast<TestMode>(test_type);
		if (testMode == TestMode::Read && static_cast<uint8_t>(prompt("0 for S1, 1 for Pro")) == 1) {
			board = std::make_shared<can::CANBoard>(robot::types::boardid_t::debug2, device);
		}

		while (true) {
			if (testMode == TestMode::State) {
				std::stringstream state_msg("");
				state_msg << "Enter desired motor state:\n";
				state_msg << static_cast<int>(can::motor::axis_state_t::idle) << " idle\n";
				state_msg << static_cast<int>(can::motor::axis_state_t::full_calib) << " full calibration\n";
				state_msg << static_cast<int>(can::motor::axis_state_t::closed_loop_control) << " closed loop control\n";
				state_msg << static_cast<int>(can::motor::axis_state_t::lockin_spin) << " lockin spin\n";
				state_msg << static_cast<int>(can::motor::axis_state_t::harmonic_calib) << " harmonic calibration";

				int state = prompt(state_msg.str().c_str());
				can::motor::axis_state_t motor_state = static_cast<can::motor::axis_state_t>(state);
				board->setMotorState(motor_state);
			} else if (testMode == TestMode::Power) {
				// TODO: float function
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
					can::addDirectReadCallback(board->getDevice(), endpoint_id, [&](auto decoded, std::unique_lock<std::shared_mutex> lock) {
						if (!lock.mutex()) { return; }

						std::string type = endpoint["type"];
						printf("%s from 0x%02X [%s]: ", input.c_str(), board->getDevice().deviceUUID, type.c_str());
						if (type == "uint32") {
							printf("%u\n", decoded.value_uint32);
						} else if (type == "int32") {
							printf("%i\n", decoded.value_int32);
						} else if (type == "uint16") {
							printf("%u\n", decoded.value_uint16);
						} else if (type == "uint8") {
							printf("%u\n", decoded.value_uint8);
						} else if (type == "float") {
							printf("%f\n", decoded.value_float);
						} else if (type == "bool") {
							printf("%s\n", decoded.value_bool ? "true" : "false");
						}

						can::removeDirectReadCallback(board->getDevice(), endpoint["id"], std::move(lock));
					}, true);

					board->read(endpoint_id);
					std::this_thread::sleep_for(std::chrono::milliseconds(500));
				} else {
					std::cout << "Unknown endpoint" << std::endl;
					continue;
				}
			} else if (testMode == TestMode::Write) {
				std::string input;
				std::cout << "Enter endpoint name" << " > ";
				std::getline(std::cin, input);

				if (nlohmann::json endpoint = can::getEndpoint(board->getBoardID(), input); endpoint != nullptr) {
					uint16_t endpoint_id = endpoint["id"];
					uint32_t value = prompt("value");
					board->write(endpoint, value);
				}
			} else if (testMode == TestMode::Stepper) {
				std::string input;
				std::cout << "Enter revs: ";
				std::getline(std::cin, input);
				float revs = std::stof(input);
				CANPacket_t packet = CANMotorPacket_Stepper_DriveRevolutions(Constants::JETSON_DEVICE, device, revs);
				can::printCANPacket(packet);
				can::sendCANPacket(packet);
			} else if (testMode == TestMode::Peripheral) {
				uint8_t periphID = prompt("peripheral ID");
				std::string input;
				std::cout << "Enter pwm duty cycle: ";
				std::getline(std::cin, input);
				float dutyCycle = std::stof(input);
				CANPacket_t packet = CANPeripheralPacket_SetPWMDutyCycle(Constants::JETSON_DEVICE, device, periphID, dutyCycle);
				can::sendCANPacket(packet);
			} else if (testMode == TestMode::RawCAN) {
				uint8_t pr = prompt("priority");
				uint8_t command = prompt("command");
				uint8_t dlc = prompt("add'l. data bits");
				if (dlc > 5) {
					std::cout << "Too many data bits" << std::endl;
					continue;
				}
				uint8_t data[dlc + 1];
				data[0] = command;

				for (int i = 1; i <= dlc; i++) {
					data[i] = prompt("bit " + std::to_string(i));
				}

				// // manual construction of a generic packet
				CANPacket_t p = {};
				p.device = device;
				p.priority = static_cast<CANPriority_t>(pr);
				p.command = command;
				p.senderUUID = CAN_UUID_JETSON;
				p.contentsLength = dlc;
				for (int i = 0; i < p.contentsLength && i < 6; i++) {
					p.contents[i] = data[i + 1];
				}

				can::printCANPacket(p);
				can::sendCANPacket(p);
			} else if (testMode == TestMode::Debug) {
				CANPacket_t p = CANPeripheralPacket_SetBrakes(
					Constants::JETSON_DEVICE, CANDevice_t{1, 0, 0, CAN_UUID_TELEMETRY}, brake + 1, 0
				);
				can::sendCANPacket(p);

				brake = (brake + 1) % 6;

				std::this_thread::sleep_for(std::chrono::milliseconds(500));

				// double avg = 0;
				// double worst = 0;
				// auto last = std::chrono::steady_clock::now();
				// int n = 1;

				// can::addDirectReadCallback(board->getDevice(), 374, [&board, &avg, &worst, &last, &n]([[maybe_unused]] auto decoded, [[maybe_unused]] std::unique_lock<std::shared_mutex> lock) {
				// 	auto recv = std::chrono::steady_clock::now();
				// 	double diff = std::chrono::duration<double, std::milli>(recv - last).count();
				// 	diff = std::fmod(diff, 50);
				// 	avg += (diff - avg) / n;
				// 	if (diff > worst) {
				// 		worst = diff;
				// 		printf("%d: %.3f ms, avg: %.3f ms, worst: %.3f ms\n", n, diff, avg, worst);
				// 	} else if (n % 1000 == 0) {
				// 		printf("%d: %.3f ms, avg: %.3f ms, worst: %.3f ms\n", n, diff, avg, worst);
				// 	}

				// 	last = recv;
				// 	n += 1;

				// 	board->read(374);
				// }, true);

				// board->read(374);
				// while(true);
			}
		}
	}
}
