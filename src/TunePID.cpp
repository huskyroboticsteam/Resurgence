#include "CAN/CAN.h"
#include "CAN/CANMotor.h"
#include "log.h"
#include "world_interface/real_world_constants.h"
#include "world_interface/world_interface.h"
#include "Constants.h"

#include <csignal>
#include <ctime>
#include <iostream>
#include <map>
#include <time.h>
#include <unistd.h>

#include <sys/time.h>

extern "C" {
#include "HindsightCAN/CANCommon.h"
}

using namespace robot::types;

template <typename K, typename V> std::map<V, K> reverseMap(const std::map<K, V>& map) {
	std::map<V, K> reversed;
	for (const auto& [k, v] : map) {
		reversed.emplace(v, k);
	}
	return reversed;
}

const std::map<motorid_t, std::string> motorNameMap = {
	{motorid_t::frontLeftWheel, "frontLeftWheel"},
	{motorid_t::frontRightWheel, "frontRightWheel"},
	{motorid_t::rearLeftWheel, "rearLeftWheel"},
	{motorid_t::rearRightWheel, "rearRightWheel"},
	{motorid_t::armBase, "armBase"},
	{motorid_t::shoulder, "shoulder"},
	{motorid_t::elbow, "elbow"},
	{motorid_t::forearm, "forearm"},
	{motorid_t::wrist, "wrist"},
	{motorid_t::hand, "hand"},
	{motorid_t::activeSuspension, "activeSuspension"}};

const std::map<std::string, motorid_t> nameToMotorMap = reverseMap(motorNameMap);

const uint8_t motor_group_id = DEVICE_GROUP_MOTOR_CONTROL;

void cleanup(int signum) {
	std::cout << "Interrupted\n";
	can::motor::emergencyStopMotors();
	exit(0);
}

int main(int argc, char** argv) {
	robot::world_interface_init();
	struct timeval tp0, tp_start;

	// TODO: Before running this script, make sure the PPJR is set correctly for each motor
	// in real_world_constants.cpp

	signal(SIGINT, cleanup);

	std::cout << "Motor Names:" << std::endl;
	for (const auto& motor : robot::pidMotors) {
		auto name = motorNameMap.at(motor);
		std::cout << "\t" << name << std::endl;
	}
	std::cout << "Enter motor name > ";
	std::string motor_name;
	std::getline(std::cin, motor_name);
	motorid_t motor = nameToMotorMap.at(motor_name);
	uint8_t serial = robot::motorSerialIDMap.at(motor);

	printf("Enter coefficients for motor [%s] (serial %d):\n", motor_name.c_str(), serial);
	std::string str;
	std::cout << "P > ";
	std::getline(std::cin, str);
	int p_coeff = std::stoi(str);
	std::cout << "I > ";
	std::getline(std::cin, str);
	int i_coeff = std::stoi(str);
	std::cout << "D > ";
	std::getline(std::cin, str);
	int d_coeff = std::stoi(str);

	can::motor::setMotorMode(serial, can::motor::motormode_t::pid);
	can::motor::setMotorPIDConstants(serial, p_coeff, i_coeff, d_coeff);

	double timestep = 0.0;
	double period = 2.0;
	double amplitude =
		10 * 1000.0; // in 1000th's of degrees
					 // (doesn't make sense for hand motor, but that doesn't use PID)

	usleep(300 * 1000); // wait for encoder position data to arrive
	int32_t starting_angle = can::motor::getMotorPosition(serial);
	int32_t angle_target = starting_angle;
	double acc_error = 0.0;
	int total_steps = 0;

	while (total_steps < 60) {
		gettimeofday(&tp_start, NULL);

		int32_t current_angle = can::motor::getMotorPosition(serial);
		double difference = (current_angle - angle_target) / 1000.0;
		acc_error += difference * difference;
		printf("Step %02d: target %05d actual %05d\n", total_steps, angle_target,
			   current_angle);
		total_steps += 1;

		timestep += 1.0 / Constants::CONTROL_HZ;
		angle_target =
			(int32_t)round(amplitude * sin(2 * M_PI * timestep / period)) + starting_angle;

		robot::setMotorPos(motor, angle_target);

		gettimeofday(&tp0, NULL);
		long elapsedUsecs =
			(tp0.tv_sec - tp_start.tv_sec) * 1000 * 1000 + (tp0.tv_usec - tp_start.tv_usec);
		long desiredUsecs = 1000 * 1000 / Constants::CONTROL_HZ;
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
