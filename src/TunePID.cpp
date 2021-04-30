#include <time.h>
#include <sys/time.h>
#include <ctime>
#include <csignal>
#include <unistd.h>
#include <iostream>

#include "Rover.h"
#include "Globals.h"
#include "simulator/world_interface.h"
#include "Networking/CANUtils.h"
#include "Networking/ParseCAN.h"

extern "C"
{
    #include "HindsightCAN/CANMotorUnit.h"
    #include "HindsightCAN/CANSerialNumbers.h"
    #include "HindsightCAN/CANCommon.h"
    #include "HindsightCAN/CANPacket.h"
}

const double CONTROL_HZ = 10.0;
const uint8_t motor_group_id = DEVICE_GROUP_MOTOR_CONTROL;

void cleanup(int signum) {
  std::cout << "Interrupted\n";
  CANPacket p;
  AssembleGroupBroadcastingEmergencyStopPacket(&p, motor_group_id, ESTOP_ERR_GENERAL);
  sendCANPacket(p);
  usleep(1000); // Give the packet time to be sent
  exit(0);
}

int main(int argc, char** argv)
{
    world_interface_init();
    InitializeRover(MOTOR_UNIT_MODE_PID);
    struct timeval tp0, tp_start;

    // TODO: Before running this script, make sure the PPJR is set correctly for each motor
    // in Rover.cpp.

    signal(SIGINT, cleanup);

    std::string str;
    std::cout << "Enter motor serial > ";
    std::getline(std::cin, str);
    int raw_serial = std::stoi(str);
    uint8_t serial = (uint8_t) serial;
    std::string motor_name = motor_group[serial];
    std::cout << "Enter coefficients for motor " << motor_name << ":\n";
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
    double period = 3.0;
    double amplitude = 15 * 1000.0; // in 1000th's of degrees
            // (doesn't make sense for hand motor, but that doesn't use PID)
    int32_t angle_target = 0;
    double acc_error = 0.0;
    int total_steps = 0;

    while (total_steps < 30)
    {
        gettimeofday(&tp_start, NULL);

        while (recvCANPacket(&p) != 0) {
            ParseCANPacket(p);
        }

        int32_t current_angle = Globals::status_data[motor_name]["angular_position"];
        double difference = (current_angle - angle_target) / 1000.0;
        acc_error += difference * difference;
        printf("Step %02d: target %05d actual %05d\n", total_steps, angle_target, current_angle);
        total_steps += 1;

        timestep += 1.0/CONTROL_HZ;
        angle_target = (int32_t) round(amplitude * sin(2*M_PI * timestep / period));

        AssemblePIDTargetSetPacket(&p, motor_group_id, serial, angle_target);
        sendCANPacket(p);

        gettimeofday(&tp0, NULL);
        long elapsedUsecs = (tp0.tv_sec - tp_start.tv_sec) * 1000 * 1000 + (tp0.tv_usec - tp_start.tv_usec);
        long desiredUsecs = 1000 * 1000 / CONTROL_HZ;
        if (desiredUsecs - elapsedUsecs > 0) {
            usleep(desiredUsecs - elapsedUsecs);
        } else {
            std::cout << "Can't keep up with control frequency! Desired " << desiredUsecs/1000 << " elapsed " << elapsedUsecs/1000 << std::endl;
        }
    }
    return 0;
}
