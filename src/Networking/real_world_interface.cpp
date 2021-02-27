#include <iostream>
#include "../simulator/world_interface.h"
#include "../simulator/utils.h"
#include "../Globals.h"
#include "json.hpp"
#include "motor_interface.h"
#include "CANUtils.h"

extern "C"
{
    #include "../HindsightCAN/CANMotorUnit.h"
}

using nlohmann::json;

void world_interface_init() {
  return;
}

const double WHEEL_BASE = 1.0; // Distance between left and right wheels. Eyeballed
const double WHEEL_RADIUS = 0.15; // Eyeballed
const double PWM_FOR_1RAD_PER_SEC = 10000; // Eyeballed

bool setCmdVel(double dtheta, double dx)
{
  if (Globals::E_STOP && (dtheta != 0 || dx != 0)) return false;

  /* This is the inverse of the formula:
   *    dx = (right_ground_vel + left_ground_vel) / 2
   *    dtheta = (right_ground_vel - left_ground_vel) / WHEEL_BASE
   */
  double right_ground_vel = dx + WHEEL_BASE/2*dtheta;
  double left_ground_vel = dx - WHEEL_BASE/2*dtheta;
  double right_angular_vel = right_ground_vel / WHEEL_RADIUS;
  double left_angular_vel = left_ground_vel / WHEEL_RADIUS;
  int16_t right_pwm = (int16_t) (right_angular_vel * PWM_FOR_1RAD_PER_SEC);
  int16_t left_pwm = (int16_t) (left_angular_vel * PWM_FOR_1RAD_PER_SEC);
  // This is a bit on the conservative side, but we heard an ominous popping sound at 20000.
  int16_t max_pwm = 15000;
  if (abs(right_pwm) > max_pwm) {
    std::cout << "WARNING: requested too-large right PWM " << right_pwm << std::endl;
    right_pwm = max_pwm*(right_pwm < 0 ? -1 : 1);
  }
  if (abs(left_pwm) > max_pwm) {
    std::cout << "WARNING: requested too-large left PWM " << left_pwm << std::endl;
    left_pwm = max_pwm*(left_pwm < 0 ? -1 : 1);
  }

  CANPacket p;
  uint8_t motor_group = 0x04;
  AssemblePWMDirSetPacket(&p, motor_group, DEVICE_SERIAL_MOTOR_CHASSIS_FL, left_pwm);
  sendCANPacket(p);
  AssemblePWMDirSetPacket(&p, motor_group, DEVICE_SERIAL_MOTOR_CHASSIS_FR, right_pwm);
  sendCANPacket(p);
  AssemblePWMDirSetPacket(&p, motor_group, DEVICE_SERIAL_MOTOR_CHASSIS_BL, left_pwm);
  sendCANPacket(p);
  AssemblePWMDirSetPacket(&p, motor_group, DEVICE_SERIAL_MOTOR_CHASSIS_BR, right_pwm);
  sendCANPacket(p);
  return true;
}

// Everything below this line still needs to be implemented

points_t readLidarScan() {
  return {};
}

points_t readLandmarks() {
  return {};
}

transform_t readGPS() {
  return toTransform({0,0,0});
}

transform_t readOdom() {
  return toTransform({0,0,0});
}

URCLeg getLeg(int index) {
  return URCLeg { -1, -1, {0.,0.,0.}};
}
