#include "Rover.h"

#include "Autonomous.h"
#include "CommandLineOptions.h"
#include "Globals.h"
#include "Networking/CANUtils.h"
#include "Networking/Network.h"
#include "Networking/NetworkConstants.h"
#include "Networking/ParseBaseStation.h"
#include "Networking/ParseCAN.h"
#include "Networking/motor_interface.h"
#include "Networking/mc/MissionControlProtocol.h"
#include "Util.h"
#include "navtypes.h"
#include "log.h"
#include "rospub.h"
#include "world_interface/world_interface.h"

#include <array>
#include <chrono>
#include <csignal>
#include <ctime>
#include <fstream>
#include <sstream>
#include <thread>
#include <time.h>
#include <unistd.h>

#include <sys/time.h>

extern "C" {
#include "HindsightCAN/CANCommon.h"
#include "HindsightCAN/CANMotorUnit.h"
#include "HindsightCAN/CANSerialNumbers.h"
}

using std::chrono::duration_cast;
using std::chrono::microseconds;
using std::chrono::steady_clock;
using namespace navtypes;
void initEncoders(bool zero_encoders) {
	CANPacket p;
	for (uint8_t serial = DEVICE_SERIAL_MOTOR_BASE;
		 serial < DEVICE_SERIAL_MOTOR_HAND; // The hand motor doesn't have an encoder
		 serial++) {
		uint8_t encoder_sign = Constants::arm_encoder_signs[serial - 1];
		AssembleEncoderInitializePacket(&p, DEVICE_GROUP_MOTOR_CONTROL, serial, 0,
										encoder_sign, zero_encoders);
		sendCANPacket(p);
		usleep(1000); // We're running out of CAN buffer space
		AssembleEncoderPPJRSetPacket(&p, DEVICE_GROUP_MOTOR_CONTROL, serial,
									 Constants::arm_PPJRs[serial - 1]);
		sendCANPacket(p);
		usleep(1000); // We're running out of CAN buffer space
		AssembleTelemetryTimingPacket(&p, DEVICE_GROUP_MOTOR_CONTROL, serial,
									  PACKET_TELEMETRY_ANG_POSITION, 100); // 100 ms, for 10 hz
		sendCANPacket(p);
		usleep(1000); // We're running out of CAN buffer space
	}
}

void setArmMode(uint8_t mode) {
	// Set all arm motors to given mode
	for (uint8_t serial = DEVICE_SERIAL_MOTOR_BASE; serial <= DEVICE_SERIAL_MOTOR_HAND;
		 serial++) {
		setMotorMode(serial, mode);
	}
}

void setMotorMode(uint8_t serial, uint8_t mode) {
	if ((serial > DEVICE_SERIAL_MOTOR_ELBOW || serial < 1) && mode == MOTOR_UNIT_MODE_PID) {
		log(LOG_ERROR, "Cannot use PID mode for motor serial %d\n", serial);
		return;
	}
	CANPacket p;
	AssembleModeSetPacket(&p, DEVICE_GROUP_MOTOR_CONTROL, serial, mode);
	sendCANPacket(p);
	usleep(1000); // We're running out of CAN buffer space
	if (mode == MOTOR_UNIT_MODE_PID) {
		int p_coeff = Constants::arm_Ps[serial - 1];
		int i_coeff = Constants::arm_Is[serial - 1];
		int d_coeff = Constants::arm_Ds[serial - 1];
		AssemblePSetPacket(&p, DEVICE_GROUP_MOTOR_CONTROL, serial, p_coeff);
		sendCANPacket(p);
		AssembleISetPacket(&p, DEVICE_GROUP_MOTOR_CONTROL, serial, i_coeff);
		sendCANPacket(p);
		AssembleDSetPacket(&p, DEVICE_GROUP_MOTOR_CONTROL, serial, d_coeff);
		sendCANPacket(p);
		usleep(1000); // We're running out of CAN buffer space
	}
}

void InitializeRover(uint8_t arm_mode, bool zero_encoders) {
	InitializeCANSocket();

	// Set all wheel motors to mode PWM
	CANPacket p;
	for (uint8_t serial = DEVICE_SERIAL_MOTOR_CHASSIS_FL;
		 serial <= DEVICE_SERIAL_MOTOR_CHASSIS_BR; serial++) {
		AssembleModeSetPacket(&p, DEVICE_GROUP_MOTOR_CONTROL, serial, MOTOR_UNIT_MODE_PWM);
		sendCANPacket(p);
		usleep(1000); // We're running out of CAN buffer space
	}

	initEncoders(zero_encoders);
	// Weird bug on AVR boards. Without this delay, the AVR boards won't move the motors when
	// we use PWM control later. (This problem does not arise if we do not call initEncoders.)
	usleep(1 * 1000 * 1000);
	setArmMode(arm_mode);
}

void closeRover(int signum) {
	rospub::shutdown();
	Globals::websocketServer.stop();
	raise(SIGTERM);
}

std::vector<URCLegGPS> parseGPSLegs(std::string filepath) {
	std::vector<URCLegGPS> urc_legs;
	std::ifstream gps_legs(filepath);

	int left_post_id, right_post_id;
	double lat, lon;
	std::string line;

	// A properly formatted file has each leg on a separate line, with each line of the form
	// left_post_id right_post_id lat lon
	// Improperly formatted lines (empty lines, comments) are ignored, and text
	// after the above data on properly formatted lines is ignored
	// An example can be found at example_gps_legs.txt
	while (getline(gps_legs, line)) {
		std::istringstream line_stream(line);
		if (line_stream >> left_post_id >> right_post_id >> lat >> lon) {
			// we assume that gps already has a fix
			gpscoords_t gps = {lat, lon};
			URCLegGPS leg = {left_post_id, right_post_id, gps};
			log(LOG_INFO, "Got urc leg at lat=%f lon=%f\n", lat, lon);
			urc_legs.push_back(leg);
		}
	}
	log(LOG_INFO, "Got %d urc legs\n", urc_legs.size());

	if (urc_legs.size() == 0) {
		log(LOG_ERROR, "could not get URC legs\n");
		exit(0);
	}

	return urc_legs;
}

int rover_loop(int argc, char** argv) {
	LOG_LEVEL = LOG_INFO;
	Globals::AUTONOMOUS = false;
	Globals::websocketServer.start();
	mc::MissionControlProtocol mcProto(Globals::websocketServer);
	Globals::websocketServer.addProtocol(mcProto);
	world_interface_init();
	rospub::init();
	// Ctrl+C doesn't stop the simulation without this line
	signal(SIGINT, closeRover);
	Globals::opts = ParseCommandLineOptions(argc, argv);
	InitializeRover(MOTOR_UNIT_MODE_PWM, true);
	CANPacket packet;

	// Target locations for autonomous navigation
	// Eventually this will be set by communication from the base station
	std::vector<URCLegGPS> urc_legs = parseGPSLegs("../src/gps/simulator_legs.txt");
	Autonomous autonomous(urc_legs, CONTROL_HZ);
	char buffer[MAXLINE];
	auto roverStart = steady_clock::now();
	int num_can_packets = 0;
	for (int iter = 0; /*no termination condition*/; iter++) {
		auto loopStart = steady_clock::now();
		long loopStartElapsedUsecs =
			duration_cast<microseconds>(loopStart - roverStart).count();
		num_can_packets = 0;
		while (recvCANPacket(&packet) != 0) {
			num_can_packets += 1;
			ParseCANPacket(packet);
		}
		log(LOG_DEBUG, "Got %d CAN packets\n", num_can_packets);

		int arm_base_pos = -1;
		int shoulder_pos = -1;
		int elbow_pos = -1;
		if (!Globals::status_data["arm_base"]["angular_position"].is_null())
			arm_base_pos = Globals::status_data["arm_base"]["angular_position"];
		if (!Globals::status_data["shoulder"]["angular_position"].is_null())
			shoulder_pos = Globals::status_data["shoulder"]["angular_position"];
		if (!Globals::status_data["elbow"]["angular_position"].is_null())
			elbow_pos = Globals::status_data["elbow"]["angular_position"];
		log(LOG_DEBUG, "Time\t %d arm_base\t %d\t shoulder\t %d\t elbow\t %d \r",
			loopStartElapsedUsecs / 1000, arm_base_pos, shoulder_pos, elbow_pos);

		// TODO make this not depend on the old protocol
		//incrementArm();

		autonomous.autonomyIter();

		long elapsedUsecs =
			duration_cast<microseconds>(steady_clock::now() - loopStart).count();
		long desiredUsecs = 1000 * 1000 / CONTROL_HZ;
		if (desiredUsecs - elapsedUsecs > 0) {
			// We drift by approximately 1ms per second unless we
			// reduce our sleep time slightly
			usleep(desiredUsecs - elapsedUsecs - 90);
		} else {
			log(LOG_WARN, "Can't keep up with control frequency! Desired %d elapsed %d\n",
				desiredUsecs / 1000, elapsedUsecs / 1000);
		}
	}
	return 0;
}
