#include "CAN.h"
#include "CANMotor.h"
#include "CANUtils.h"
#include "../control/JacobianVelController.h"
#include "../navtypes.h"
#include "../utils/core.h"
#include "../utils/scheduler.h"
#include "../world_interface/data.h"

#include <algorithm>
#include <chrono>
#include <iostream>
#include <mutex>
#include <thread>
#include <unordered_set>
#include <vector>

extern "C" {
#include <HindsightCAN/CANPower.h>
#include <HindsightCAN/CANScience.h>
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

namespace {

class CANBoard {
public:
	CANBoard(robot::types::boardid_t motor, bool hasPosSensor,
					can::uuid_t uuid, can::domainmask_t domains,
					double pos_pwm_scale, double neg_pwm_scale)
	: motor_id(motor),
	has_pos_sensor(hasPosSensor),
	board_uuid(uuid),
	board_domains(domains),
	positive_scale(pos_pwm_scale),
	negative_scale(neg_pwm_scale) {
		// create scheduler if needed
		std::lock_guard<std::mutex> lg(schedulerMutex);
		if (!pSched) {
			pSched.emplace("MotorVelSched");
		}
	}

	void setMotorPower(double power) {
		ensureMotorMode(can::motor::motormode_t::pwm);

		// scale the power
		double scale = power < 0 ? negative_scale : positive_scale;
		power *= scale;
		can::motor::setMotorPower(board_uuid, board_domains, power);
	}

	void setMotorPos(int32_t targetPos) {
		ensureMotorMode(can::motor::motormode_t::pid);
		can::motor::setMotorPIDTarget(board_uuid, board_domains, targetPos);
	}

	robot::types::DataPoint<int32_t> getMotorPos() const {
		return can::motor::getMotorPosition(board_uuid, board_domains);
	}

	void setMotorVel(int32_t targetVel) {
		ensureMotorMode(can::motor::motormode_t::pid);
		if (!velController) {
			constructVelController();
		}
		// set velocity target
		navtypes::Vectord<1> velocityVector{targetVel};
		robot::types::datatime_t currTime = robot::types::dataclock::now();
		velController->setTarget(currTime, velocityVector);

		// check to see if the event exists. if yes, unschedule it
		unscheduleVelocityEvent();

		// schedule position event
		velEventID = pSched->scheduleEvent(100ms, [this]() -> void {
			robot::types::datatime_t currTime = robot::types::dataclock::now();
			auto motorPos = getMotorPos();
			if (motorPos.isValid()) {
				const navtypes::Vectord<1> currPos(motorPos.getData());
				navtypes::Vectord<1> posCommand = velController->getCommand(currTime, currPos);
				setMotorPos(posCommand.coeff(0, 0));
			}
		});
	}
	
	void unscheduleVelocityEvent() {
		if (velEventID) {
			pSched->removeEvent(velEventID.value());
			velEventID.reset();
		}
	}

	can::uuid_t getMotorUUID() const {
		return board_uuid;
	}

	robot::types::boardid_t getMotorID() const {
		return motor_id;
	}

private:
	robot::types::boardid_t motor_id;
	bool has_pos_sensor;
	can::uuid_t board_uuid;
	can::domainmask_t board_domains;
	std::optional<can::motor::motormode_t> motor_mode;
	double positive_scale;
	double negative_scale;
	std::optional<util::PeriodicScheduler<std::chrono::steady_clock>::eventid_t> velEventID;
	std::optional<JacobianVelController<1, 1>> velController;
	
	inline static std::optional<util::PeriodicScheduler<std::chrono::steady_clock>> pSched;
	inline static std::mutex schedulerMutex;

	void ensureMotorMode(can::motor::motormode_t mode) {
		if (!motor_mode || motor_mode.value() != mode) {
			// update the motor mode
			motor_mode.emplace(mode);
			can::motor::setMotorMode(board_uuid, board_domains, mode);
		}
	}	

	void constructVelController() {
		// define dimensions
		constexpr int32_t inputDim = 1;
		constexpr int32_t outputDim = 1;

		// create kinematics function (input and output will both be the current motor
		// position)
		const std::function<navtypes::Vectord<outputDim>(const navtypes::Vectord<inputDim>&)>&
			kinematicsFunct = [](const navtypes::Vectord<inputDim>& inputVec) {
				// returns a copy of the input vector
				return inputVec;
			};

		// create jacobian function (value will be 1 since it's the derivative of the
		// kinematics function)
		const std::function<navtypes::Matrixd<outputDim, inputDim>(
			const navtypes::Vectord<inputDim>&)>& jacobianFunct =
			[](const navtypes::Vectord<inputDim>&) {
				navtypes::Matrixd<outputDim, inputDim> res =
					navtypes::Matrixd<outputDim, inputDim>::Identity();
				return res;
			};

		velController.emplace(kinematicsFunct, jacobianFunct);
	}
};
} // namespace robot

std::unordered_set<int> modes = {
	static_cast<int>(TestMode::ModeSet),   static_cast<int>(TestMode::PWM),
	static_cast<int>(TestMode::PID),	   static_cast<int>(TestMode::Encoder),
	static_cast<int>(TestMode::PIDVel),	   static_cast<int>(TestMode::LimitSwitch),
	static_cast<int>(TestMode::Telemetry), static_cast<int>(TestMode::ScienceServos),
  static_cast<int>(TestMode::Stepper), static_cast<int>(TestMode::RawCAN)};

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
			auto group = static_cast<can::devicegroup_t>(prompt("Enter group code"));
			int serial = prompt("Enter motor serial");
			int mode = prompt("Enter mode (0 for PWM, 1 for PID)");
			std::cout << "got " << serial << " and " << mode << std::endl;
			can::motor::setMotorMode(group, serial,
									 mode == 0 ? motormode_t::pwm : motormode_t::pid);
		} else if (testMode == TestMode::PWM) {
			auto group = static_cast<can::devicegroup_t>(prompt("Enter group code"));
			int serial = prompt("Enter motor serial");
			int pwm = prompt("Enter PWM");
			can::motor::setMotorMode(group, serial, motormode_t::pwm);
			can::motor::setMotorPower(group, serial, static_cast<int16_t>(pwm));
		} else if (testMode == TestMode::PID) {
			static can::devicegroup_t group;
			static int serial;
			if (!mode_has_been_set) {
				group = static_cast<can::devicegroup_t>(prompt("Enter group code"));
				serial = prompt("Enter motor serial");
				// AVR board firmware resets the angle target every time it receives a
				// mode set packet, so we only want to send this once.
				// TODO: do we need to set the PPJR?
				can::motor::setMotorMode(group, serial, motormode_t::pid);
				mode_has_been_set = true;
			}

			int p_coeff = prompt("P");
			int i_coeff = prompt("I");
			int d_coeff = prompt("D");

			can::motor::setMotorPIDConstants(group, serial, p_coeff, i_coeff, d_coeff);

			int angle_target = prompt("Enter PID target (in 1000ths of degrees)");
			can::motor::setMotorPIDTarget(group, serial, angle_target);
		} else if (testMode == TestMode::PIDVel) {
			static robot::types::datatime_t startTime;
			static int32_t targetVel;
			static std::shared_ptr<CANBoard> motor;
			static robot::types::DataPoint<int32_t> initialMotorPos;
			static double vel_timeout;

			if (!mode_has_been_set) {
				can::uuid_t uuid = static_cast<can::uuid_t>(prompt("Enter device UUID"));
				can::domainmask_t domains = static_cast<can::domainmask_t>(prompt("Enter domain mask"));

				// set pid mode
				can::motor::setMotorMode(uuid, domains, motormode_t::pid);
				// can::motor::setMotorMode(group, serial, motormode_t::pid);
				mode_has_been_set = true;

				// get pid coeffs
				int p_coeff = prompt("P");
				int i_coeff = prompt("I");
				int d_coeff = prompt("D");

				// set pid coeffs
				can::motor::setMotorPIDConstants(uuid, domains, p_coeff, i_coeff, d_coeff);
				// can::motor::setMotorPIDConstants(group, serial, p_coeff, i_coeff, d_coeff);

				// create can motor
				double posScale =
					prompt("Enter the positive scale for the motor (double value)\n");
				double negScale =
					prompt("Enter the negative scale for the motor (double value)\n");
				motor = std::make_shared<CANBoard>(boardid_t::leftTread, true, uuid,
														   domains, posScale, negScale);

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
		} else if (testMode == TestMode::Encoder) {
			static can::devicegroup_t group;
			static int serial;
			if (!mode_has_been_set) {
				group = static_cast<can::devicegroup_t>(prompt("Enter group code"));
				serial = prompt("Enter motor serial");

				int sensorType;
				do {
					sensorType =
						prompt("What type of sensor?\n0 for encoder\n1 for potentiometer\n");
				} while (sensorType != 0 && sensorType != 1);

				std::chrono::milliseconds telemPeriod(prompt("Telemetry period (ms)"));

				can::motor::initMotor(group, serial);
				if (sensorType == 0) {
					int ppjr = prompt("Pulses per joint revolution");
					bool invert = prompt("Invert? 1=yes, 0=no") == 1;
					can::motor::initEncoder(group, serial, invert, true, ppjr, telemPeriod);
				} else if (sensorType == 1) {
					int posLo = prompt("Pos Lo");
					int posHi = prompt("Pos Hi");
					int adcLo = prompt("ADC Lo");
					int adcHi = prompt("ADC Hi");
					can::motor::initPotentiometer(group, serial, posLo, posHi, adcLo, adcHi,
												  telemPeriod);
				}
				mode_has_been_set = true;
			}
			auto encoderData = can::motor::getMotorPosition(group, serial);
			std::string encoderStr =
				encoderData ? std::to_string(encoderData.getData()) : "null";
			// \33[2k is the ANSI escape sequence for erasing the current console line
			// the output is a single changing line instead of flooding the console with text
			std::cout << "\33[2K\rEncoder value: " << encoderStr << std::flush;
			std::this_thread::sleep_for(20ms);
		} else if (testMode == TestMode::LimitSwitch) {
			static bool testLimits = false;
			static can::devicegroup_t group;
			static int serial;
			if (!mode_has_been_set) {
				group = static_cast<can::devicegroup_t>(prompt("Enter group code"));
				serial = prompt("Enter motor serial");
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
		} else if (testMode == TestMode::Telemetry) {
			if (!mode_has_been_set) {
				auto group = static_cast<can::devicegroup_t>(prompt("Enter group code"));
				int serial = prompt("Enter serial");
				can::deviceid_t deviceID = std::make_pair(group, serial);
				auto telemType = static_cast<can::telemtype_t>(prompt("Enter telemetry type"));
				can::addDeviceTelemetryCallback(
					deviceID, telemType,
					[](can::deviceid_t id, can::telemtype_t telemType,
					   DataPoint<can::telemetry_t> data) {
						std::cout << "Telemetry: group=" << std::hex
								  << static_cast<int>(id.first) << ", serial=" << std::hex
								  << static_cast<int>(id.second) << ", type=" << std::hex
								  << static_cast<int>(telemType) << ", data=" << std::dec
								  << data.getDataOrElse(0) << std::endl;
					});
				int telemPeriod = prompt("Telemetry timing (ms)");
				bool useTimingPacket =
					static_cast<bool>(prompt("What telemetry method?\n0 for pull packets\n1 "
											 "for telemetry timing packet"));
				if (useTimingPacket) {
					CANPacket packet;
					AssembleTelemetryTimingPacket(
						&packet, static_cast<uint8_t>(deviceID.first), deviceID.second,
						static_cast<uint8_t>(telemType), telemPeriod);
					can::sendCANPacket(packet);
				} else {
					can::scheduleTelemetryPull(deviceID, telemType,
											   std::chrono::milliseconds(telemPeriod));
				}
				mode_has_been_set = true;
			}
			std::this_thread::sleep_for(1s);
		} else if (testMode == TestMode::ScienceServos) {
			int servo_no = prompt("Enter servo no");
			int degrees = prompt("Enter degrees");

			CANPacket p;
			AssembleScienceServoPacket(&p, 0x7, 0x5, (uint8_t)servo_no,
									   (uint8_t)degrees);
			can::sendCANPacket(p);
      can::printCANPacket(p);
		} else if (testMode == TestMode::Stepper) {
      int stepper = prompt("Enter stepper");
      int angle = prompt("Enter angle");

      CANPacket p;
      AssembleScienceStepperTurnAnglePacket(&p, 0x7, 0x4, stepper, angle, 0x3);
      can::sendCANPacket(p);
      can::printCANPacket(p);
    } else if (testMode == TestMode::RawCAN) {
      uint8_t pr = prompt("priority");
      uint8_t g = prompt("group");
      uint8_t s = prompt("serial");
      uint8_t pid = prompt("pid");
      uint8_t dlc = prompt("add'l. data bits");
      uint8_t data[dlc+1];
      data[0] = pid;

      for (int i = 1; i <= dlc; i++) {
        data[i] = prompt("bit");
      }

      CANPacket p;
      AssembleCANPacket(&p, pr, g, s, pid, dlc+1, data);
      can::sendCANPacket(p);
      can::printCANPacket(p);
    }
	}
}
