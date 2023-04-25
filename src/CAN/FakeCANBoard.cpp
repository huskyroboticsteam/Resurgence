
#include "CAN.h"
#include "CANMotor.h"
#include "CANUtils.h"

#include <algorithm>
#include <chrono>
#include <iostream>
#include <thread>
#include <unordered_set>
#include <vector>

extern "C" {
#include "../HindsightCAN/CANScience.h"
}

using namespace std::chrono_literals;

using can::motor::motormode_t;
using namespace robot::types;

enum class TestMode {
	ModeSet,
	PWM,
	PID,
	Encoder,
	LimitSwitch,
	ScienceTelemetry,
	ScienceMotors,
	ScienceServos
};

std::unordered_set<int> modes = {
	static_cast<int>(TestMode::ModeSet),	   static_cast<int>(TestMode::PWM),
	static_cast<int>(TestMode::PID),		   static_cast<int>(TestMode::Encoder),
	static_cast<int>(TestMode::LimitSwitch),   static_cast<int>(TestMode::ScienceTelemetry),
	static_cast<int>(TestMode::ScienceMotors), static_cast<int>(TestMode::ScienceServos)};

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

	CANPacket p;
	uint8_t science_group = static_cast<int>(can::devicegroup_t::science);
	std::stringstream ss("What are you testing?\n");
	ss << static_cast<int>(TestMode::ModeSet) << " for MODE SET\n";
	ss << static_cast<int>(TestMode::PWM) << " for PWM\n";
	ss << static_cast<int>(TestMode::PID) << " for PID\n";
	ss << static_cast<int>(TestMode::Encoder) << " for ENCODER\n";
	ss << static_cast<int>(TestMode::LimitSwitch) << " for LIMIT SWITCH\n";
	ss << static_cast<int>(TestMode::ScienceTelemetry) << " for SCIENCE TELEMETRY\n";
	ss << static_cast<int>(TestMode::ScienceMotors) << " for SCIENCE MOTORS\n";
	ss << static_cast<int>(TestMode::ScienceTelemetry) << " for SCIENCE SERVOS\n";
	int test_type = prompt(ss.str().c_str());
	if (modes.find(test_type) == modes.end()) {
		std::cout << "Unrecognized response: " << test_type << std::endl;
		std::exit(1);
	}
	TestMode testMode = static_cast<TestMode>(test_type);
	int serial = 0;
	if (testMode == TestMode::PWM || testMode == TestMode::PID ||
		testMode == TestMode::Encoder || testMode == TestMode::LimitSwitch) {
		serial = prompt("Enter motor serial");
	}
	bool mode_has_been_set = false;

	while (true) {
		if (testMode == TestMode::ModeSet) {
			serial = prompt("Enter motor serial");
			int mode = prompt("Enter mode (0 for PWM, 1 for PID)");
			std::cout << "got " << serial << " and " << mode << std::endl;
			can::motor::setMotorMode(serial, mode == 0 ? motormode_t::pwm : motormode_t::pid);
		} else if (testMode == TestMode::PWM) {
			int pwm = prompt("Enter PWM");
			can::motor::setMotorMode(serial, motormode_t::pwm);
			can::motor::setMotorPower(serial, static_cast<int16_t>(pwm));
		} else if (testMode == TestMode::PID) {
			// Don't send all five packets at once. On some motor boards, the CAN buffer
			// only fits four packets.

			if (!mode_has_been_set) {
				// AVR board firmware resets the angle target every time it receives a
				// mode set packet, so we only want to send this once.
				// TODO: do we need to set the PPJR?
				can::motor::setMotorMode(serial, motormode_t::pid);
				mode_has_been_set = true;
			}

			int p_coeff = prompt("P");
			int i_coeff = prompt("I");
			int d_coeff = prompt("D");

			can::motor::setMotorPIDConstants(serial, p_coeff, i_coeff, d_coeff);

			int angle_target = prompt("Enter PID target (in 1000ths of degrees)");
			can::motor::setMotorPIDTarget(serial, angle_target);
		} else if (testMode == TestMode::Encoder) {
			if (!mode_has_been_set) {
				int sensorType;
				do {
					sensorType =
						prompt("What type of sensor?\n0 for encoder\n1 for potentiometer\n");
				} while (sensorType != 0 && sensorType != 1);

				std::chrono::milliseconds telemPeriod(prompt("Telemetry period (ms)"));

				can::motor::initMotor(serial);
				if (sensorType == 0) {
					int ppjr = prompt("Pulses per joint revolution");
					can::motor::initEncoder(serial, false, true, ppjr, telemPeriod);
				} else if (sensorType == 1) {
					int posLo = prompt("Pos Lo");
					int posHi = prompt("Pos Hi");
					int adcLo = prompt("ADC Lo");
					int adcHi = prompt("ADC Hi");
					can::motor::initPotentiometer(serial, posLo, posHi, adcLo, adcHi, telemPeriod);
				}
				mode_has_been_set = true;
			}
			auto encoderData = can::motor::getMotorPosition(serial);
			std::string encoderStr =
				encoderData ? std::to_string(encoderData.getData()) : "null";
			// \33[2k is the ANSI escape sequence for erasing the current console line
			// the output is a single changing line instead of flooding the console with text
			std::cout << "\33[2K\rEncoder value: " << encoderStr << std::flush;
			std::this_thread::sleep_for(20ms);
		} else if (testMode == TestMode::LimitSwitch) {
			static bool testLimits = false;
			if (!mode_has_been_set) {
				testLimits = static_cast<bool>(prompt("Set limits? 1=yes,0=no"));
				if (testLimits) {
					int lo = prompt("Low position");
					int hi = prompt("High position");
					std::chrono::milliseconds telemPeriod(prompt("Telemetry period (ms)"));
					int ppjr = prompt("Pulses per joint revolution");
					can::motor::initEncoder(serial, false, true, ppjr, telemPeriod);
					can::motor::setLimitSwitchLimits(serial, lo, hi);
				} else {
					can::deviceid_t id = std::make_pair(can::devicegroup_t::motor, serial);
					can::addDeviceTelemetryCallback(
						id, can::telemtype_t::limit_switch,
						[](can::deviceid_t id, can::telemtype_t telemType,
						   DataPoint<can::telemetry_t> data) {
							std::cout << "Motor Limit: serial=" << std::hex
									  << static_cast<int>(id.second)
									  << ", data=" << std::bitset<8>(data.getDataOrElse(0))
									  << std::endl;
						});
				}
				mode_has_been_set = true;
			}
			if (testLimits) {
				auto encoderData = can::motor::getMotorPosition(serial);
				std::string encoderStr =
					encoderData ? std::to_string(encoderData.getData()) : "null";
				std::cout << "\33[2K\rEncoder value: " << encoderStr << std::flush;
				std::this_thread::sleep_for(20ms);
			}
		} else if (testMode == TestMode::ScienceTelemetry) {
			// Serial 0 seems to work. Nothing else (up to 17, the largest I tried)
			serial = prompt("Enter serial");
			uint8_t sensor_code = 42; // Nonsense for now, just testing CAN connection
			AssembleTelemetryPullPacket(&p, science_group, (uint8_t)serial, sensor_code);
			can::sendCANPacket(p);
		} else if (testMode == TestMode::ScienceMotors) {
			// CAREFUL, there are no safety checks / limit switches
			// 0: drill up/down (down is positive, ~500 PWM)
			// 1: drawer open/close (open is positive, ~100 PWM)
			// 2: drill rotate (clockwise/down is positive, ~300 PWM with no soil)
			int motor_no = prompt("Enter motor no");
			int pwm = prompt("Enter pwm");
			can::motor::setMotorMode(motor_no, motormode_t::pwm);
			can::motor::setMotorPower(motor_no, static_cast<int16_t>(pwm));
		} else if (testMode == TestMode::ScienceServos) {
			int servo_no = prompt("Enter servo no");
			int degrees = prompt("Enter degrees");
			AssembleScienceServoPacket(&p, science_group, 0x0, (uint8_t)servo_no,
									   (uint8_t)degrees);
			can::sendCANPacket(p);
		}
	}
}
