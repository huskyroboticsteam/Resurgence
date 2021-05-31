
#include <string>
#include <time.h>
#include <sys/time.h>

#include "../log.h"
#include "../Rover.h"
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
const std::array<std::string, 4> operation_modes = {
	"PID target", "PWM target", "incremental PID speed", "incremental IK speed"};

// For PWM control, we want to use higher values when acting against gravity
const std::map<std::string, double> positive_arm_pwm_scales = {
	{"arm_base",    6000},
	{"shoulder",   20000},
	{"elbow",     -31000},
	{"forearm",    -4000},
	{"diffleft",    5000},
	{"diffright",  -5000},
	{"hand",       15000}
};
const std::map<std::string, double> negative_arm_pwm_scales = {
	{"arm_base",    6000},
	{"shoulder",   12000},
	{"elbow",     -14000},
	{"forearm",    -4000},
	{"diffleft",    5000},
	{"diffright",  -5000},
	{"hand",       15000}
};
const std::map<std::string, double> incremental_pid_scales = {
	{"arm_base",  M_PI/8}, // TODO: Check signs
	{"shoulder",  M_PI/8},
	{"elbow",     M_PI/8},
	{"forearm",        0}, // We haven't implemented PID on these motors yet
	{"diffleft",       0},
	{"diffright",      0},
	{"hand",           0}
};
constexpr double incremental_ik_scale = 0.1; // m/s

std::vector<std::string> ik_axes = {"IK_X", "IK_Y", "IK_Z"};
const std::array<std::string, 3> ik_motors = {"arm_base", "shoulder", "elbow"};

constexpr int INCREMENTAL_TIMEOUT_MS = 300;

int getIndex(const std::vector<std::string> &arr, const std::string &value)
{
  for (int i = 0; i < arr.size(); i++) {
    if (arr[i] == value) {
      return i;
    }
  }
  return -1;
}

bool isValidOperationMode(int motor_serial, int ik_axis, const std::string &op_mode)
{
  if (op_mode != "PWM target" && (motor_serial > 3 || motor_serial < 1) && ik_axis < 0)
  {
    return false;
  }
  else if ((op_mode != "incremental IK speed") ^ (ik_axis < 0))
  {
    return false;
  }
  return true;
}

// `motor` here is required to be an actual motor (e.g. `arm_base`)
bool setMotorOperationMode(const std::string &motor, const std::string &op_mode) {
  int motor_serial = getIndex(motor_group, motor);
  std::string prev_mode = "PWM target";
  json prev_mode_json = Globals::status_data[motor]["operation_mode"];
  if (!prev_mode_json.is_null()) prev_mode = prev_mode_json;
  if (prev_mode != op_mode) {
    log(LOG_INFO, "Changing operation mode for %s (%s -> %s)\n",
        motor.c_str(), prev_mode.c_str(), op_mode.c_str());
    Globals::status_data[motor]["operation_mode"] = op_mode;
    if (op_mode == "PWM target") {
      setMotorMode(motor_serial, MOTOR_UNIT_MODE_PWM);
    } else if (prev_mode == "PWM target") {
      bool missing_encoder = Globals::status_data[motor]["angular_position"].is_null();
      if (missing_encoder) return false;
      setMotorMode(motor_serial, MOTOR_UNIT_MODE_PID);
    }
  }
  return true;
}

// `motor` here could be an IK axis rather than an actual motor
bool setOperationMode(const std::string &motor, int ik_axis, const std::string &op_mode)
{
  bool success = true;
  if (ik_axis < 0) {
    success &= setMotorOperationMode(motor, op_mode);
  } else {
    for (const std::string &physical_motor : ik_motors) {
      success &= setMotorOperationMode(physical_motor, op_mode);
    }
  }
  struct timeval now;
  gettimeofday(&now, NULL);
  long now_ms = now.tv_sec * 1000 + now.tv_usec / 1000;
  Globals::status_data[motor]["most_recent_command"] = now_ms;
  return success;
}

void sendPWMPacket(const std::string &motor, double normalized_pwm)
{
  CANPacket p;
  auto& scale_map = (normalized_pwm > 0 ? positive_arm_pwm_scales : negative_arm_pwm_scales);
  double pwm = normalized_pwm * scale_map.at(motor);
  int motor_serial = getIndex(motor_group, motor);
  AssemblePWMDirSetPacket(&p, DEVICE_GROUP_MOTOR_CONTROL, motor_serial, pwm);
  sendCANPacket(p);
}

void sendPIDPacket(const std::string &motor, int32_t pid_target)
{
  CANPacket p;
  int motor_serial = getIndex(motor_group, motor);
  AssemblePIDTargetSetPacket(&p, DEVICE_GROUP_MOTOR_CONTROL, motor_serial, pid_target);
  sendCANPacket(p);
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
  int ik_axis = getIndex(ik_axes, motor);
  if (motor_serial < 0 && ik_axis < 0)
  {
    return sendError("Unrecognized motor " + motor);
  }

  log(LOG_DEBUG, "Parsing motor packet for motor %s\n", motor.c_str());

  for (const std::string key : operation_modes)
  {
    if (message[key] == nullptr) continue;
    if (!isValidOperationMode(motor_serial, ik_axis, key))
    {
      return sendError("Invalid operation mode '" + key + "' for motor " + motor);
    }

    if (!setOperationMode(motor, ik_axis, key)) {
      return sendError("Cannot use PID for motor " + motor + ": No encoder data yet");
    }

    // TODO: rename this key. We're not actually treating it as the literal PWM value.
    if (key == "PWM target")
    {
      double normalized_pwm = message[key];
      sendPWMPacket(motor, normalized_pwm);
    }
    else if (key == "PID target")
    {
      int32_t pid_target = message[key];
      sendPIDPacket(motor, pid_target);
    }
    else if (key == "incremental PID speed")
    {
      int current_angle = Globals::status_data[motor]["angular_position"];
      int current_millideg = 0;
      json curr_md = Globals::status_data[motor]["millideg_per_control_loop"];
      if (!curr_md.is_null()) current_millideg = curr_md;
      double speed = message[key];
      int millideg_per_control_loop = (incremental_pid_scales.at(motor) * 1000) * speed / CONTROL_HZ;
      if (current_millideg != millideg_per_control_loop) {
        Globals::status_data[motor]["millideg_per_control_loop"] = millideg_per_control_loop;
        Globals::status_data[motor]["target_angle"] = current_angle;
        log(LOG_DEBUG, "Setting incremental PID for %s to %d starting from %d\n",
            motor.c_str(), millideg_per_control_loop, current_angle);
      }
    }
    else if (key == "incremental IK speed")
    {
      // TODO assemble IK packet and call the method from Networking/IK
      // This requires implementing forward kinematics so I'll do it in a separate PR
      return sendError("IK is not yet supported");
    }
  }

  return true;
}

// We assume this method will be called once per control loop.
void incrementArmPID()
{
  for (const std::string &motor : motor_group)
  {
    json op_mode = Globals::status_data[motor]["operation_mode"];
    if (!op_mode.is_null() && op_mode == "incremental PID speed")
    {
      struct timeval now;
      gettimeofday(&now, NULL);
      long now_ms = now.tv_sec * 1000 + now.tv_usec / 1000;
      long most_recent_ms = Globals::status_data[motor]["most_recent_command"];
      if (now_ms - most_recent_ms > INCREMENTAL_TIMEOUT_MS)
      {
        // timeout for safety
        Globals::status_data[motor]["millideg_per_control_loop"] = 0;
        return;
      }

      int mdpl = Globals::status_data[motor]["millideg_per_control_loop"];
      int target = Globals::status_data[motor]["target_angle"];
      int new_target = target + mdpl;
      Globals::status_data[motor]["target_angle"] = new_target;
      sendPIDPacket(motor, new_target);
    }
  }
}

void incrementArm()
{
  incrementArmPID();
  // incrementArmIK(); // TODO implement
}
