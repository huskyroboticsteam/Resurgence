
#include "CANUtils.h"
#include "TestPackets.h"
#include <iostream>
extern "C"
{
    #include "../HindsightCAN/CANMotorUnit.h"
}

constexpr int TEST_MODE_SET = 0;
constexpr int TEST_PWM = 1;
constexpr int TEST_PID = 2;

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
  int test_type = prompt("What are you testing? (0 for MODE SET, 1 for PWM, 2 for PID)");
  int serial = prompt("Enter motor serial");
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
  }
}
