#include <time.h>
#include <sys/time.h>
#include <ctime>
#include <csignal>
#include <unistd.h>

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
    #include "HindsightCAN/CANSerialNumbers.h"
    #include "HindsightCAN/CANCommon.h"
}

void initEncoders()
{
    CANPacket p;
    for (uint8_t serial = DEVICE_SERIAL_MOTOR_BASE;
        serial < DEVICE_SERIAL_MOTOR_HAND; // The hand motor doesn't have an encoder
        serial ++ ) {
      AssembleEncoderInitializePacket(&p, DEVICE_GROUP_MOTOR_CONTROL, serial,
          0, 0, 1); // 1 means to zero the encoder angle measurement
      sendCANPacket(p);
      usleep(1000); // We're running out of CAN buffer space
      AssembleEncoderPPJRSetPacket(   &p, DEVICE_GROUP_MOTOR_CONTROL, serial,
          1024); // I have no idea how many pulses actually make one rotation
      sendCANPacket(p);
      usleep(1000); // We're running out of CAN buffer space
      AssembleTelemetryTimingPacket(  &p, DEVICE_GROUP_MOTOR_CONTROL, serial,
          PACKET_TELEMETRY_ANG_POSITION, 100); // 100 ms, for 10 hz
      sendCANPacket(p);
      usleep(1000); // We're running out of CAN buffer space
    }
}

void setArmMode(uint8_t mode)
{
    // Set all arm motors to given mode
    CANPacket p;
    for (uint8_t serial = DEVICE_SERIAL_MOTOR_BASE;
        serial <= DEVICE_SERIAL_MOTOR_HAND;
        serial ++ ) {
      AssembleModeSetPacket(&p, DEVICE_GROUP_MOTOR_CONTROL, serial, mode);
      sendCANPacket(p);
      usleep(1000); // We're running out of CAN buffer space
    }
}

void InitializeRover(uint8_t arm_mode)
{
    InitializeCANSocket();

    // Set all wheel motors to mode PWM
    CANPacket p;
    uint8_t mode_PWM = 0x0;
    for (uint8_t serial = DEVICE_SERIAL_MOTOR_CHASSIS_FL;
        serial <= DEVICE_SERIAL_MOTOR_CHASSIS_BR;
        serial ++) {
      AssembleModeSetPacket(&p, DEVICE_GROUP_MOTOR_CONTROL, serial, MOTOR_UNIT_MODE_PWM);
      sendCANPacket(p);
      usleep(1000); // We're running out of CAN buffer space
    }

    initEncoders();
    // Weird bug on AVR boards. Without this delay, the AVR boards won't move the motors when
    // we use PWM control later. (This problem does not arise if we do not call initEncoders.)
    usleep(1 * 1000 * 1000); 
    setArmMode(arm_mode);
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
    InitializeRover(MOTOR_UNIT_MODE_PWM);
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
        while (recvCANPacket(&packet) != 0) {
            ParseCANPacket(packet);
        }
        InitializeBaseStationSocket();
        bzero(buffer, sizeof(buffer));
        while (recvBaseStationPacket(buffer) != 0) {
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
