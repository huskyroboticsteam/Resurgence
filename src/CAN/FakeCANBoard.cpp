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
using namespace robot::types;

enum class TestMode {
	ModeSet,
	PWM,
	PID,
	PIDVel,
	Encoder,
	LimitSwitch,
	Telemetry,
	ScienceServos,
	Stepper,
	RawCAN
};

std::unordered_set<int> modes = {
	static_cast<int>(TestMode::ModeSet),   static_cast<int>(TestMode::PWM),
	static_cast<int>(TestMode::PID),	   static_cast<int>(TestMode::Encoder),
	static_cast<int>(TestMode::PIDVel),	   static_cast<int>(TestMode::LimitSwitch),
	static_cast<int>(TestMode::Telemetry), static_cast<int>(TestMode::ScienceServos),
	static_cast<int>(TestMode::Stepper),   static_cast<int>(TestMode::RawCAN)};

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
	ss << static_cast<int>(TestMode::PWM) << " for PWM\n";
	ss << static_cast<int>(TestMode::PID) << " for PID\n";
	ss << static_cast<int>(TestMode::Encoder) << " for ENCODER\n";
	ss << static_cast<int>(TestMode::LimitSwitch) << " for LIMIT SWITCH\n";
	ss << static_cast<int>(TestMode::Telemetry) << " for TELEMETRY\n";
	ss << static_cast<int>(TestMode::ScienceServos) << " for SCIENCE SERVOS\n";
	ss << static_cast<int>(TestMode::Stepper) << " for STEPPER\n";
	ss << static_cast<int>(TestMode::RawCAN) << " for CAN\n";
	int test_type = prompt(ss.str().c_str());
	if (modes.find(test_type) == modes.end()) {
		std::cout << "Unrecognized response: " << test_type << std::endl;
		std::exit(1);
	}
	TestMode testMode = static_cast<TestMode>(test_type);
	bool mode_has_been_set = false;

	while (true) {
		if (testMode == TestMode::ModeSet) {
			int uuid = static_cast<uint16_t>(prompt("Enter device uuid"));
			int mode = prompt("Enter mode (0 for PWM, 1 for PID)");

			CANDevice_t device;
			device.deviceUUID = uuid;
			std::cout << "got " << device.deviceUUID << std::endl;
			can::motor::setMotorMode(device, mode == 0 ? motormode_t::vel : motormode_t::pos);
		} else if (testMode == TestMode::PWM) {
			int uuid = static_cast<uint16_t>(prompt("Enter device uuid"));

			CANDevice_t device;
			device.deviceUUID = uuid;

			CANPacket_t p;
			while (true) {
				int vel = prompt("vel");
				if (vel == 0) {
					p = CANMotorPacket_BLDC_SetAxisState(Constants::JETSON_DEVICE, device, BLDC_AXIS_IDLE);
					can::sendCANPacket(p);
					break;
				}
				// int dur = prompt("dur (s)");

				p = CANMotorPacket_BLDC_SetInputVelocity(Constants::JETSON_DEVICE, device, vel, 0);
				can::sendCANPacketWithAck(p);

				p = CANMotorPacket_BLDC_SetAxisState(Constants::JETSON_DEVICE, device, BLDC_AXIS_IDLE);
				can::sendCANPacket(p);
				// std::cout << "Lockin Spin..." << std::endl;
				p = CANMotorPacket_BLDC_SetAxisState(Constants::JETSON_DEVICE, device, BLDC_AXIS_LOCKIN_SPIN);
				can::sendCANPacket(p);
				// std::this_thread::sleep_for(std::chrono::seconds(dur));

				// std::cout << "Stopping..." << std::endl;
				// p = CANMotorPacket_BLDC_SetInputVelocity(Constants::JETSON_DEVICE, device, 0, 0);
				// can::sendCANPacket(p);

			}
		} else if (testMode == TestMode::PID) {
			/*
			static CANDevice_t device;

			if (!mode_has_been_set) {
				uuid = prompt("Enter device uuid");
				// AVR board firmware resets the angle target every time it receives a
				// mode set packet, so we only want to send this once.
				// TODO: do we need to set the PPJR?
				device.deviceUUID = uuid;
				can::motor::setMotorMode(device, motormode_t::pid);
				mode_has_been_set = true;
			}

			int p_coeff = prompt("P");
			int i_coeff = prompt("I");
			int d_coeff = prompt("D");

			can::motor::setMotorPIDConstants(device, p_coeff, i_coeff, d_coeff);

			int angle_target = prompt("Enter PID target (in 1000ths of degrees)");
			can::motor::setMotorPIDTarget(device, angle_target);
			*/

		} else if (testMode == TestMode::PIDVel) {
			/*
			static robot::types::datatime_t startTime;
			static int32_t targetVel;
			static std::shared_ptr<CANBoard> motor;
			static robot::types::DataPoint<int32_t> initialMotorPos;
			static double vel_timeout;

			if (!mode_has_been_set) {
				CANDevice_t device = prompt("Enter device UUID");

				// set pid mode
				can::motor::setMotorMode(device, motormode_t::pid);
				mode_has_been_set = true;

				// get pid coeffs
				int p_coeff = prompt("P");
				int i_coeff = prompt("I");
				int d_coeff = prompt("D");

				// set pid coeffs
				can::motor::setMotorPIDConstants(device, p_coeff, i_coeff, d_coeff);
				// can::motor::setMotorPIDConstants(device, p_coeff, i_coeff, d_coeff);

				// create can motor
				double posScale =
					prompt("Enter the positive scale for the motor (double value)\n");
				double negScale =
					prompt("Enter the negative scale for the motor (double value)\n");
				motor = std::make_shared<CANBoard>(boardid_t::leftTread, true, device,
			posScale, negScale);

				// get initial motor position
				DataPoint<int32_t> dataPoint = motor->getMotorPos();
				while (!dataPoint.isValid()) {
					std::this_thread::sleep_for(100ms);
					dataPoint = motor->getMotorPos();
				}
				initialMotorPos = dataPoint.getData();

				// create velocity command
				vel_timeout = prompt("Enter the number of seconds you want the command to run "
									 "for (double value)\n");
				targetVel = prompt("Enter the target velocity (in millidegrees per second)\n");
				motor->setMotorVel(targetVel);
				startTime = robot::types::dataclock::now();
			}

			// get x data: current time
			robot::types::datatime_t currTime = robot::types::dataclock::now();

			// get y data: set point (target vel * time since set velocity call) + initial pos
			double setPoint = (targetVel * util::durationToSec(currTime - startTime)) +
							  initialMotorPos.getData();
			LOG_F(INFO, "Set point: %f", setPoint);

			// get y data: motor position
			robot::types::DataPoint<int32_t> motorPos = motor->getMotorPos();
			LOG_F(INFO, "Motor position: %d", motorPos.getData());

			// check if time is up
			double elapsedTime = util::durationToSec(currTime - startTime);
			if (elapsedTime > vel_timeout) {
				// stop arm movement: set power to 0
				motor->setMotorPower(0.0);
			}
			*/
		} else if (testMode == TestMode::Encoder) {
			/* TO DO: initEncoder

			static CANDevice_t device;
			if (!mode_has_been_set) {
				device.deviceUUID = static_cast<uint16_t>prompt("Enter device");

				int sensorType;
				do {
					sensorType =
						prompt("What type of sensor?\n0 for encoder\n1 for potentiometer\n");
				} while (sensorType != 0 && sensorType != 1);

				std::chrono::milliseconds telemPeriod(prompt("Telemetry period (ms)"));

				can::motor::initMotor(device);

				if (sensorType == 0) {
					int ppjr = prompt("Pulses per joint revolution");
					bool invert = prompt("Invert? 1=yes, 0=no") == 1;

					can::motor::initEncoder(device, invert, true, ppjr, telemPeriod);
					can::motor::initEncoder(device, invert, true, telemePeriod);

				} else if (sensorType == 1) {
					int posLo = prompt("Pos Lo");
					int posHi = prompt("Pos Hi");
					int adcLo = prompt("ADC Lo");
					int adcHi = prompt("ADC Hi");
					can::motor::initPotentiometer(device, posLo, posHi, adcLo, adcHi,
												  telemPeriod);
				}
				mode_has_been_set = true;
			}
			auto encoderData = can::motor::getMotorPosition(device);
			std::string encoderStr =
				encoderData ? std::to_string(encoderData.getData()) : "null";
			// \33[2k is the ANSI escape sequence for erasing the current console line
			// the output is a single changing line instead of flooding the console with text
			std::cout << "\33[2K\rEncoder value: " << encoderStr << std::flush;
			std::this_thread::sleep_for(20ms);
			*/
		} else if (testMode == TestMode::LimitSwitch) {
			/* TO DO: initEncoder, setLimitSwitchLimits

			static bool testLimits = false;
			static CANDevice_t device;
			if (!mode_has_been_set) {
				CANDevice_t device = prompt("Enter device");
				testLimits = static_cast<bool>(prompt("Set limits? 1=yes,0=no"));
				if (testLimits) {
					int lo = prompt("Low position");
					int hi = prompt("High position");
					std::chrono::milliseconds telemPeriod(prompt("Telemetry period (ms)"));
					int ppjr = prompt("Pulses per joint revolution");
					can::motor::initEncoder(group, serial, false, true, ppjr, telemPeriod);
					can::motor::setLimitSwitchLimits(group, serial, lo, hi);
				} else {
					can::deviceid_t id = std::make_pair(group, serial);
					can::addDeviceTelemetryCallback(
						id, can::telemtype_t::limit_switch,
						[](can::deviceid_t id, [[maybe_unused]] can::telemtype_t telemType,
						   DataPoint<can::telemetry_t> data) {
							std::cout << "Motor Limit: group=" << std::hex
									  << static_cast<int>(id.first) << ", serial=" << std::hex
									  << static_cast<int>(id.second)
									  << ", data=" << std::bitset<8>(data.getDataOrElse(0))
									  << std::endl;
						});
				}
				mode_has_been_set = true;
			}
			if (testLimits) {
				auto encoderData = can::motor::getMotorPosition(group, serial);
				std::string encoderStr =
					encoderData ? std::to_string(encoderData.getData()) : "null";
				std::cout << "\33[2K\rEncoder value: " << encoderStr << std::flush;
				std::this_thread::sleep_for(20ms);
			}
			*/
		} else if (testMode == TestMode::Telemetry) {
			if (!mode_has_been_set) {
				CANDeviceUUID_t uuid = static_cast<uint16_t>(prompt("Enter device uuid"));
				auto telemType = static_cast<can::telemtype_t>(prompt("Enter telemetry type"));
				can::addDeviceTelemetryCallback(
					uuid, telemType,
					[](can::uuid_t uuid, can::telemtype_t telemType,
					   DataPoint<can::telemetry_t> data) {
						std::cout << "Telemetry: uuid=" << static_cast<int>(uuid)
								  << ", type=" << static_cast<int>(telemType)
								  << static_cast<int>(telemType) << ", data=" << std::dec
								  << data.getDataOrElse(0) << std::endl;
					});
				int telemPeriod = prompt("Telemetry timing (ms)");
				bool useTimingPacket =
					static_cast<bool>(prompt("What telemetry method?\n0 for pull packets\n1 "
											 "for telemetry timing packet"));
				if (useTimingPacket) {
					/* TO DO: Telemetry Packets

					CANPacket packet;
					AssembleTelemetryTimingPacket(
						&packet, static_cast<uint8_t>(deviceID.first), deviceID.second,
						static_cast<uint8_t>(telemType), telemPeriod);
					can::sendCANPacket(packet);
					*/
				} else {
					can::scheduleTelemetryPull(uuid, telemType,
											   std::chrono::milliseconds(telemPeriod));
				}
				mode_has_been_set = true;
			}
			std::this_thread::sleep_for(1s);
		} else if (testMode == TestMode::ScienceServos) {
			/* TO DO: Remove science-related from codebase

			int servo_no = prompt("Enter servo no");
			int degrees = prompt("Enter degrees");

			CANPacket p;
			AssembleScienceServoPacket(&p, 0x7, 0x5, (uint8_t)servo_no,
									   (uint8_t)degrees);
			can::sendCANPacket(p);
			can::printCANPacket(p); */
		} else if (testMode == TestMode::Stepper) {
			/*
				int stepper = prompt("Enter stepper");
		  int angle = prompt("Enter angle");

		  CANPacket_t p;
		  AssembleScienceStepperTurnAnglePacket(&p, 0x7, 0x4, stepper, angle, 0x3);
		  can::sendCANPacket(p);
		  can::printCANPacket(p);
		  */
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