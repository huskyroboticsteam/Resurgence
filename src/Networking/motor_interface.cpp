
#include <string>
#include "../log.h"
#include "CANUtils.h"
#include "ParseCAN.h"
extern "C"
{
    #include "../HindsightCAN/CANMotorUnit.h"
}
#include "../Globals.h"

#include "../simulator/world_interface.h"
#include "../simulator/utils.h"
#include "ParseBaseStation.h"
#include "motor_interface.h"

// It's important that all of these vectors are sorted in the same order (the indices correspond)
std::map<std::string, std::vector<std::string>> possible_keys = {
	{"motor", {"PID target", "PWM target"}},
};

// For PWM control, we want to use higher values when acting against gravity
std::map<std::string, double> positive_arm_pwm_scales = {
	{"arm_base",    6000},
	{"shoulder",   20000},
	{"elbow",     -31000},
	{"forearm",    -4000},
	{"diffleft",    5000},
	{"diffright",  -5000},
	{"hand",       15000}
};
std::map<std::string, double> negative_arm_pwm_scales = {
	{"arm_base",    6000},
	{"shoulder",   12000},
	{"elbow",     -14000},
	{"forearm",    -4000},
	{"diffleft",    5000},
	{"diffright",  -5000},
	{"hand",       15000}
};

int getIndex(const std::vector<std::string> &arr, std::string &value)
{
  for (int i = 0; i < arr.size(); i++) {
    if (arr[i] == value) {
      return i;
    }
  }
  return -1;
}

bool ParseMotorPacket(json &message)
{
  if (Globals::E_STOP) return sendError("Emergency stop is activated");

  std::string motor;
  try
  {
    motor = message["motor"];
  }
  catch (json::type_error)
  {
    return sendError("No motor specified");
  }
  int motor_serial = getIndex(motor_group, motor);
  if (motor_serial < 0)
  {
    return sendError("Unrecognized motor " + motor);
  }

  log(LOG_DEBUG, "Parsing motor packet for motor %s", motor);

  for (int key_idx = 0; key_idx < possible_keys["motor"].size(); key_idx++)
  {
    std::string key = possible_keys["motor"][key_idx];
    if (message[key] == nullptr) continue;
    // TODO: rename this key. We're not actually treating it as the literal PWM value.
    if (key == "PWM target")
    {
      CANPacket p;
      double normalized_pwm = message[key];
      auto& scale_map = (normalized_pwm > 0 ? positive_arm_pwm_scales : negative_arm_pwm_scales);
      double pwm = normalized_pwm * scale_map[motor];
      AssemblePWMDirSetPacket(&p, DEVICE_GROUP_MOTOR_CONTROL, motor_serial, pwm);
      sendCANPacket(p);
    }
    else if (key == "PID target")
    {
      CANPacket p;
      int32_t pid_target = message[key];
      AssemblePIDTargetSetPacket(&p, DEVICE_GROUP_MOTOR_CONTROL, motor_serial, pid_target);
      sendCANPacket(p);
    }
  }

  return true;
}
