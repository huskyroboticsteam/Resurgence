#include <iostream>
#include "../simulator/world_interface.h"
#include "../simulator/utils.h"
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

const double WHEEL_BASE = 0.8; // eyeballed
const double WHEEL_RADIUS = 0.2; // eyeballed
const double PWM_FOR_1RAD_PER_SEC = 8000; // eyeballed
/*
dx = (right + left) / 2
dtheta = (right - left) / (distance bw wheels)

therefore:

2*dx + (distance bw wheels) * dtheta = 2*right
2*dx - (distance bw wheels) * dtheta = 2*left

now "right" gives the ground velocity of the wheel, so
right_angular_vel = right / wheel_radius

right_pwm = right_angular_vel * PWM_FOR_1RAD_PER_SEC

*/

bool setCmdVel(double dtheta, double dx)
{
  double lr = dtheta;
  double fb = dx;
  // TODO I'm curious how intuitive it would be to use remote control with wheel velocities rather
  // than kinematic velocities. That would actually allow the rover to go faster, I think.
  // Another alternative: add a boolean "wheel_velocities" indicating which mode to use
  // (if true, the other keys should just be "right" and "left").
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
