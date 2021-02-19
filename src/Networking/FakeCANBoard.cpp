
#include "CANUtils.h"
#include "TestPackets.h"
#include <iostream>
extern "C"
{
    #include "../HindsightCAN/CANMotorUnit.h"
}

const bool TEST_MODE_SET = false;
const bool TEST_PWM = true;

int main() {
  InitializeCANSocket();

  std::string str;
  CANPacket p;
  uint8_t motor_group = 0x04;
  if (TEST_PWM) {
    AssembleModeSetPacket(&p, motor_group, (uint8_t) 0x0, (uint8_t) 0x0);
    sendCANPacket(p);
  }
  while(1) {
    if (TEST_MODE_SET) {
      std::cout << "Enter motor serial > ";
      std::getline(std::cin, str);
      int serial = std::stoi(str);
      std::cout << "Enter mode (0 for PWM, 1 for PID) > ";
      std::getline(std::cin, str);
      int mode = std::stoi(str);
      std::cout << "got " << serial << " and " << mode << std::endl;
      AssembleModeSetPacket(&p, motor_group, (uint8_t) serial, (uint8_t) mode);
      sendCANPacket(p);
    }
    if (TEST_PWM) {
      std::cout << "Enter motor serial > ";
      std::getline(std::cin, str);
      int serial = std::stoi(str);
      std::cout << "Enter PWM > ";
      std::getline(std::cin, str);
      int pwm = std::stoi(str);
      AssemblePWMDirSetPacket(&p, motor_group, (uint8_t) serial, (int16_t) pwm);
      sendCANPacket(p);
    }
  }
}
