#include "../simulator/world_interface.h"
#include "../simulator/utils.h"
#include "json.hpp"
#include "motor_interface.h"
using nlohmann::json;

void world_interface_init() {
  return;
}

bool setCmdVel(double dtheta, double dx)
{
  // TODO do we need to scale or invert these values?
  double lr = dtheta;
  double fb = dx;
  // TODO I'm curious how intuitive it would be to use remote control with wheel velocities rather
  // than kinematic velocities. That would actually allow the rover to go faster, I think.
  // Another alternative: add a boolean "wheel_velocities" indicating which mode to use
  // (if true, the other keys should just be "right" and "left").
  double right = (fb + lr)/2;
  double left =  (fb - lr)/2;
  int max_pwm = 1000; // TODO figure out what is the maximum value the hardware supports
  int right_pwm = (int) max_pwm * right;
  int left_pwm = (int) max_pwm * left;
  json packet = {};
  packet["type"] = "motor";
  packet["mode"] = "PWM";
  // TODO we may need to switch some of these signs
  packet["PWM target"] = right_pwm;
  packet["motor"] = "front_right_wheel";
  ParseMotorPacket(packet);
  packet["motor"] = "back_right_wheel";
  ParseMotorPacket(packet);
  packet["PWM target"] = left_pwm;
  packet["motor"] = "front_left_wheel";
  ParseMotorPacket(packet);
  packet["motor"] = "back_left_wheel";
  ParseMotorPacket(packet);
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
