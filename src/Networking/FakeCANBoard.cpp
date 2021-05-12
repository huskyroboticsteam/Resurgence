
#include "CANUtils.h"
#include "TestPackets.h"
#include <iostream>
extern "C"
{
    #include "../HindsightCAN/CANMotorUnit.h"
}

const bool TEST_MODE_SET = false;
const bool TEST_PWM = false;
const bool TEST_PID = true;

int main() {
  InitializeCANSocket();

  std::string str;
  CANPacket p;
  uint8_t motor_group = 0x04;
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
      AssembleModeSetPacket(&p, motor_group, (uint8_t) serial, (uint8_t) 0x0);
      sendCANPacket(p);
      AssemblePWMDirSetPacket(&p, motor_group, (uint8_t) serial, (int16_t) pwm);
      sendCANPacket(p);
    }
    if (TEST_PID) {
      std::cout << "Enter motor serial > ";
      std::getline(std::cin, str);
      int serial = std::stoi(str);

      std::cout << "P > ";
      std::getline(std::cin, str);
      int p_coeff = std::stoi(str);
      std::cout << "I > ";
      std::getline(std::cin, str);
      int i_coeff = std::stoi(str);
      std::cout << "D > ";
      std::getline(std::cin, str);
      int d_coeff = std::stoi(str);

      std::cout << "Enter PID target (in 1000ths of degrees) > ";
      std::getline(std::cin, str);
      int angle_target = std::stoi(str);

      CANPacket p;
      AssembleModeSetPacket(&p, motor_group, (uint8_t) serial, (uint8_t) 0x1);
      sendCANPacket(p);
      AssemblePSetPacket(&p, motor_group, (uint8_t) serial, p_coeff);
      sendCANPacket(p);
      AssembleISetPacket(&p, motor_group, (uint8_t) serial, i_coeff);
      sendCANPacket(p);
      AssembleDSetPacket(&p, motor_group, (uint8_t) serial, d_coeff);
      sendCANPacket(p);

      AssemblePIDTargetSetPacket(&p, motor_group, (uint8_t) serial, angle_target);
      sendCANPacket(p);
    }
  }
}
