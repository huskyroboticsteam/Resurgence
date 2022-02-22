#include "Globals.h"
#include "Networking/CANUtils.h"
#include "Networking/ParseCAN.h"
#include "Rover.h"
#include "log.h"
#include "world_interface/world_interface.h"

#include <csignal>
#include <ctime>
#include <iostream>
#include <time.h>
#include <unistd.h>

#include <sys/time.h>

extern "C" {
#include "HindsightCAN/CANCommon.h"
#include "HindsightCAN/CANMotorUnit.h"
#include "HindsightCAN/CANPacket.h"
#include "HindsightCAN/CANSerialNumbers.h"
}

const uint8_t motor_group_id = DEVICE_GROUP_MOTOR_CONTROL;

void cleanup(int signum) {
	std::cout << "Interrupted\n";
	CANPacket p;
	AssembleGroupBroadcastingEmergencyStopPacket(&p, motor_group_id, ESTOP_ERR_GENERAL);
	sendCANPacket(p);
	usleep(1000); // Give the packet time to be sent
	exit(0);
}

int main(int argc, char** argv) {
	LOG_LEVEL = LOG_INFO;
	robot::world_interface_init();
	InitializeRover(MOTOR_UNIT_MODE_PID, false);
	struct timeval tp0, tp_start;

	// TODO: Before running this script, make sure the PPJR is set correctly for each motor
	// in Rover.cpp.

	signal(SIGINT, cleanup);

	std::string str;
	std::cout << "Enter motor serial > ";
	std::getline(std::cin, str);
	int raw_serial = std::stoi(str);
	uint8_t serial = (uint8_t)raw_serial;
	std::string motor_name = motor_group[serial];
	printf("Enter coefficients for motor [%s] (serial %d):\n", motor_name.c_str(), serial);
	std::cout << "P > ";
	std::getline(std::cin, str);
	int p_coeff = std::stoi(str);
	std::cout << "I > ";
	std::getline(std::cin, str);
	int i_coeff = std::stoi(str);
	std::cout << "D > ";
	std::getline(std::cin, str);
	int d_coeff = std::stoi(str);

	CANPacket p;
	AssemblePSetPacket(&p, motor_group_id, serial, p_coeff);
	sendCANPacket(p);
	AssembleISetPacket(&p, motor_group_id, serial, i_coeff);
	sendCANPacket(p);
	AssembleDSetPacket(&p, motor_group_id, serial, d_coeff);
	sendCANPacket(p);

	double timestep = 0.0;
	double period = 2.0;
	double amplitude =
		10 * 1000.0; // in 1000th's of degrees
					 // (doesn't make sense for hand motor, but that doesn't use PID)

	usleep(300 * 1000); // wait for encoder position data to arrive
	while (recvCANPacket(&p) != 0) {
		ParseCANPacket(p);
	}
	int32_t starting_angle = Globals::status_data[motor_name]["angular_position"];
	int32_t angle_target = starting_angle;
	double acc_error = 0.0;
	int total_steps = 0;

	while (total_steps < 60) {
		gettimeofday(&tp_start, NULL);

		while (recvCANPacket(&p) != 0) {
			ParseCANPacket(p);
		}

		int32_t current_angle = Globals::status_data[motor_name]["angular_position"];
		double difference = (current_angle - angle_target) / 1000.0;
		acc_error += difference * difference;
		printf("Step %02d: target %05d actual %05d\n", total_steps, angle_target,
			   current_angle);
		total_steps += 1;

		timestep += 1.0 / CONTROL_HZ;
		angle_target =
			(int32_t)round(amplitude * sin(2 * M_PI * timestep / period)) + starting_angle;

		AssemblePIDTargetSetPacket(&p, motor_group_id, serial, angle_target);
		sendCANPacket(p);

		gettimeofday(&tp0, NULL);
		long elapsedUsecs =
			(tp0.tv_sec - tp_start.tv_sec) * 1000 * 1000 + (tp0.tv_usec - tp_start.tv_usec);
		long desiredUsecs = 1000 * 1000 / CONTROL_HZ;
		if (desiredUsecs - elapsedUsecs > 0) {
			usleep(desiredUsecs - elapsedUsecs);
		} else {
			std::cout << "Can't keep up with control frequency! Desired "
					  << desiredUsecs / 1000 << " elapsed " << elapsedUsecs / 1000
					  << std::endl;
		}
	}
	std::cout << "RMSE: " << sqrt(acc_error / total_steps) << std::endl;
	return 0;
}
