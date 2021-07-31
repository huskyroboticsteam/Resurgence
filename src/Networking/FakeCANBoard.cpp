
#include "CANUtils.h"
#include "TestPackets.h"
#include <iostream>
extern "C"
{
    #include "../HindsightCAN/CANMotorUnit.h"
    #include "../HindsightCAN/CANScience.h"
}

constexpr int TEST_MODE_SET = 0;
constexpr int TEST_PWM = 1;
constexpr int TEST_PID = 2;
constexpr int TEST_SCIENCE_TELEMETRY = 3;
constexpr int TEST_SCIENCE_MOTORS = 4;
constexpr int TEST_SCIENCE_SERVOS = 5;

int prompt(const char *msg) {
  std::string str;
  std::cout << msg << " > ";
  std::getline(std::cin, str);
  int val = std::stoi(str);
  return val;
}

int main() {
  InitializeCANSocket();

  CANPacket p;
  uint8_t motor_group = 0x04;
  uint8_t science_group = 0x07;
  int test_type = prompt("What are you testing?\n\
          0 for MODE SET\n\
          1 for PWM\n\
          2 for PID\n\
          3 for SCIENCE TELEMETRY\n\
          4 for SCIENCE MOTORS\n\
          5 for SCIENCE SERVOS\n");
  int serial = 0;
  if (test_type == TEST_PWM || test_type == TEST_PID) {
      serial = prompt("Enter motor serial");
  }
  bool mode_has_been_set = false;

  while(1) {

    if (test_type == TEST_MODE_SET) {
      serial = prompt("Enter motor serial");
      int mode = prompt("Enter mode (0 for PWM, 1 for PID)");
      std::cout << "got " << serial << " and " << mode << std::endl;
      AssembleModeSetPacket(&p, motor_group, (uint8_t) serial, (uint8_t) mode);
      sendCANPacket(p);
    }

    if (test_type == TEST_PWM) {
      int pwm = prompt("Enter PWM");
      AssembleModeSetPacket(&p, motor_group, (uint8_t) serial, (uint8_t) 0x0);
      sendCANPacket(p);
      AssemblePWMDirSetPacket(&p, motor_group, (uint8_t) serial, (int16_t) pwm);
      sendCANPacket(p);
    }

    if (test_type == TEST_PID) {
      // Don't send all five packets at once. On some motor boards, the CAN buffer
      // only fits four packets.

      if (!mode_has_been_set) {
          // AVR board firmware resets the angle target every time it receives a
          // mode set packet, so we only want to send this once.
          AssembleModeSetPacket(&p, motor_group, (uint8_t) serial, (uint8_t) 0x1);
          sendCANPacket(p);
          mode_has_been_set = true;
      }

      int p_coeff = prompt("P");
      int i_coeff = prompt("I");
      int d_coeff = prompt("D");

      AssemblePSetPacket(&p, motor_group, (uint8_t) serial, p_coeff);
      sendCANPacket(p);
      AssembleISetPacket(&p, motor_group, (uint8_t) serial, i_coeff);
      sendCANPacket(p);
      AssembleDSetPacket(&p, motor_group, (uint8_t) serial, d_coeff);
      sendCANPacket(p);

      int angle_target = prompt("Enter PID target (in 1000ths of degrees)");
      AssemblePIDTargetSetPacket(&p, motor_group, (uint8_t) serial, angle_target);
      sendCANPacket(p);
    }

    if (test_type == TEST_SCIENCE_TELEMETRY) {
        // Serial 0 seems to work. Nothing else (up to 17, the largest I tried)
        serial = prompt("Enter serial");
        AssembleScienceSensorPullPacket(&p, science_group, (uint8_t) 0x0, 42);
        sendCANPacket(p);
    }

    if (test_type == TEST_SCIENCE_MOTORS) {
        // CAREFUL, there are no safety checks / limit switches
        // 0: drill up/down (down is positive, ~500 PWM)
        // 1: drawer open/close (open is positive, ~100 PWM)
        // 2: drill rotate (clockwise/down is positive, ~300 PWM with no soil)
        int motor_no = prompt("Enter motor no");
        int pwm = prompt("Enter pwm");
        AssembleScienceMotorControlPacket(&p, science_group, 0x0,
                (uint8_t) motor_no, (int16_t) pwm);
        sendCANPacket(p);
    }

    if (test_type == TEST_SCIENCE_SERVOS) {
        int servo_no = prompt("Enter servo no");
        int degrees = prompt("Enter degrees");
        AssembleScienceServoPacket(&p, science_group, 0x0,
                (uint8_t) servo_no, (uint8_t) degrees);
        sendCANPacket(p);
    }
  }
}
