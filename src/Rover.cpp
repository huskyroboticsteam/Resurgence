#include <time.h>
#include <sys/time.h>
#include <ctime>
#include <csignal>

#include "CommandLineOptions.h"
#include "Globals.h"
#include "Networking/NetworkConstants.h"
#include "Networking/Network.h"
#include "Networking/CANUtils.h"
#include "Networking/ParseCAN.h"
#include "Networking/ParseBaseStation.h"
#include "Autonomous.h"
#include "simulator/world_interface.h"

extern "C"
{
    #include "HindsightCAN/CANMotorUnit.h"
}

void setArmMode(uint8_t mode)
{
    // Set all arm motors to given mode
    CANPacket p;
    for (uint8_t serial = 0x1; serial < 0x8; serial ++ ) {
      AssembleModeSetPacket(&p, DEVICE_GROUP_MOTOR_CONTROL, serial, mode);
      sendCANPacket(p);
    }
}

void InitializeRover()
{
    InitializeCANSocket();
    InitializeBaseStationSocket();

    // Set all wheel motors to mode PWM
    CANPacket p;
    uint8_t motor_group = 0x4;
    uint8_t mode_PWM = 0x0;
    for (uint8_t serial = 0x8; serial < 0xC; serial ++ ) {
      AssembleModeSetPacket(&p, DEVICE_GROUP_MOTOR_CONTROL, serial, mode_PWM);
      sendCANPacket(p);
    }

    setArmMode(mode_PWM); // Until we get encoders / PID control working
}

void closeSim(int signum)
{
    rclcpp::shutdown();
    raise(SIGTERM);
}

const double CONTROL_HZ = 10.0;

int rover_loop(int argc, char **argv)
{
    world_interface_init();
    rclcpp::init(0, nullptr);
    // Ctrl+C doesn't stop the simulation without this line
    signal(SIGINT, closeSim);
    Globals::opts = ParseCommandLineOptions(argc, argv);
    InitializeRover();
    CANPacket packet;
    // Target location for autonomous navigation
    // Eventually this will be set by communication from the base station
    int urc_leg = 5;
    Autonomous autonomous(getLeg(urc_leg), CONTROL_HZ);
    char buffer[MAXLINE];
    struct timeval tp0, tp_start;
    for(;;)
    {
        gettimeofday(&tp_start, NULL);
        if (recvCANPacket(&packet) != 0) {
            ParseCANPacket(packet);
        }
        bzero(buffer, sizeof(buffer));
        if (recvBaseStationPacket(buffer) != 0) {
            ParseBaseStationPacket(buffer);
        }
        autonomous.autonomyIter();

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
