#include <time.h>
#include <sys/time.h>
#include <ctime>
#include <csignal>
#include <unistd.h>
#include <array>
#include <queue>
#include <fstream>

#include "CommandLineOptions.h"
#include "Globals.h"
#include "log.h"
#include "Networking/NetworkConstants.h"
#include "Networking/Network.h"
#include "Networking/CANUtils.h"
#include "Networking/ParseCAN.h"
#include "Networking/ParseBaseStation.h"
#include "Autonomous.h"
#include "simulator/world_interface.h"
//#include "gps/read_usb_gps.h"

extern "C"
{
    #include "HindsightCAN/CANMotorUnit.h"
    #include "HindsightCAN/CANSerialNumbers.h"
    #include "HindsightCAN/CANCommon.h"
}

constexpr std::array<uint32_t,6> arm_PPJRs = {
    17 * 1000, // base, rough estimate

    1200 * 1000, // shoulder, estimate, TODO there's some bug with setting PPJR for this board
    36 * 1000, // elbow, rough estimate

    360 * 1000, // forearm, unmeasured
    360 * 1000, // diff_left, unmeasured
    360 * 1000 // diff_right, unmeasured
};

// So far only the shoulder and elbow have been tuned (and only roughly)
//
// base, shoulder, elbow, forearm, diff_left, diff_right
constexpr std::array<int32_t,6> arm_Ps = {
   0,    100,    500,     0,       0,         0
};
constexpr std::array<int32_t,6> arm_Is = {
   0,      0,    300,     0,       0,         0
};
constexpr std::array<int32_t,6> arm_Ds = {
   0,   1000,  10000,     0,       0,         0
};
constexpr std::array<uint8_t,6> arm_encoder_signs = {
   0,      0,      1,     0,       0,         0
};

long getElapsedUsecs(const struct timeval &tp_start) {
  struct timeval tp0;
  gettimeofday(&tp0, NULL);
  long elapsed = (tp0.tv_sec - tp_start.tv_sec) * 1000 * 1000 + (tp0.tv_usec - tp_start.tv_usec);
  return elapsed;
}

void initEncoders(bool zero_encoders)
{
    CANPacket p;
    for (uint8_t serial = DEVICE_SERIAL_MOTOR_BASE;
        serial < DEVICE_SERIAL_MOTOR_HAND; // The hand motor doesn't have an encoder
        serial ++ ) {
      uint8_t encoder_sign = arm_encoder_signs[serial-1];
      AssembleEncoderInitializePacket(&p, DEVICE_GROUP_MOTOR_CONTROL, serial,
          0, encoder_sign, zero_encoders);
      sendCANPacket(p);
      usleep(1000); // We're running out of CAN buffer space
      AssembleEncoderPPJRSetPacket(   &p, DEVICE_GROUP_MOTOR_CONTROL, serial,
          arm_PPJRs[serial-1]);
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
      if (mode == MOTOR_UNIT_MODE_PID) {
          int p_coeff = arm_Ps[serial-1];
          int i_coeff = arm_Is[serial-1];
          int d_coeff = arm_Ds[serial-1];
          AssemblePSetPacket(&p, DEVICE_GROUP_MOTOR_CONTROL, serial, p_coeff);
          sendCANPacket(p);
          AssembleISetPacket(&p, DEVICE_GROUP_MOTOR_CONTROL, serial, i_coeff);
          sendCANPacket(p);
          AssembleDSetPacket(&p, DEVICE_GROUP_MOTOR_CONTROL, serial, d_coeff);
          sendCANPacket(p);
          usleep(1000); // We're running out of CAN buffer space
      }
    }
}

void InitializeRover(uint8_t arm_mode, bool zero_encoders)
{
    InitializeCANSocket();

    // Set all wheel motors to mode PWM
    CANPacket p;
    for (uint8_t serial = DEVICE_SERIAL_MOTOR_CHASSIS_FL;
        serial <= DEVICE_SERIAL_MOTOR_CHASSIS_BR;
        serial ++) {
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

void closeSim(int signum)
{
    rclcpp::shutdown();
    raise(SIGTERM);
}

void parseGPSLegs(std::string filepath) {
	std::ifstream gps_legs(filepath);
	if (gps_legs)
	{
//		while (!gps_legs.eof())
//		{
//			// Assumes the file at filepath is properly formatted
//			// with each leg on a separate line and of the form
//			// left_post_id right_post_id lon lat
//			int left_post_id, right_post_id;
//			double lon, lat;
//			gps_legs >> left_post_id >> right_post_id >> lon >> lat;
//			point_t leg_map_space = gpsToMeters(lon, lat);
//			URCLeg leg = {left_post_id, right_post_id, leg_map_space};
//		}
		std::cout << "Using file" << std::endl;
	}
	else
	{
		// File does not exist, use simulation legs as defaults
		for (int i = 0; i < 7; i++)
		{
			urc_legs.push(getLeg(i));
		}
	}
}

const double CONTROL_HZ = 10.0;

int rover_loop(int argc, char **argv)
{
    LOG_LEVEL = LOG_INFO;
    Globals::AUTONOMOUS = true;
    world_interface_init();
    rclcpp::init(0, nullptr);
    // Ctrl+C doesn't stop the simulation without this line
    signal(SIGINT, closeSim);
    Globals::opts = ParseCommandLineOptions(argc, argv);
    InitializeRover(MOTOR_UNIT_MODE_PWM, true);
    CANPacket packet;
    // Target location for autonomous navigation
    // Eventually this will be set by communication from the base station
    std::queue<URCLeg> urc_legs = parseGPSLegs("gps_legs.txt");
    Autonomous autonomous(urc_legs, CONTROL_HZ);
    char buffer[MAXLINE];
    struct timeval tp_rover_start;
    int num_can_packets = 0;
    gettimeofday(&tp_rover_start, NULL);
    for(int iter = 0; /*no termination condition*/; iter++)
    {
        long loopStartElapsedUsecs = getElapsedUsecs(tp_rover_start);
        num_can_packets = 0;
        while (recvCANPacket(&packet) != 0) {
            num_can_packets += 1;
            ParseCANPacket(packet);
        }
        log(LOG_DEBUG, "Got %d CAN packets\n", num_can_packets);

        int arm_base_pos = -1;
        int shoulder_pos = -1;
        int elbow_pos = -1;
        if (!Globals::status_data["arm_base"].empty())
          arm_base_pos = Globals::status_data["arm_base"]["angular_position"];
        if (!Globals::status_data["shoulder"].empty())
          shoulder_pos = Globals::status_data["shoulder"]["angular_position"];
        if (!Globals::status_data["elbow"].empty())
          elbow_pos = Globals::status_data["elbow"]["angular_position"];
        log(LOG_INFO, "Time\t %d arm_base\t %d\t shoulder\t %d\t elbow\t %d \n",
                loopStartElapsedUsecs / 1000,
                arm_base_pos, shoulder_pos, elbow_pos);

        if (iter % (int) CONTROL_HZ == 0) {
          // For computation reasons, only try to do this once per second
          InitializeBaseStationSocket();
        }
        bzero(buffer, sizeof(buffer));
        while (recvBaseStationPacket(buffer) != 0) {
            ParseBaseStationPacket(buffer);
        }
        autonomous.autonomyIter();

        long elapsedUsecs = getElapsedUsecs(tp_rover_start) - loopStartElapsedUsecs;
        long desiredUsecs = 1000 * 1000 / CONTROL_HZ;
        if (desiredUsecs - elapsedUsecs > 0) {
            usleep(desiredUsecs - elapsedUsecs);
        } else {
            log(LOG_WARN, "Can't keep up with control frequency! Desired %d elapsed %d\n",
                desiredUsecs/1000, elapsedUsecs/1000);
        }
    }
    return 0;
}
