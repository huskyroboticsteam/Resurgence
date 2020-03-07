
#include "Network.h"
#include "../Globals.h"
#include "json.hpp"
#include "CANUtils.h"
#include "ParseCAN.h"
#include <cmath>
#include <tgmath.h>

extern "C"
{
    #include "../HindsightCAN/CANMotorUnit.h"
}

#include <iostream>

using nlohmann::json;

// It's important that all of these arrays are sorted in the same order (the indices correspond)
std::map<std::string, std::vector<std::string>> possible_keys = {
	{"motor", {"mode", "P", "I", "D", "PID target", "PWM target"}},
        {"ik", {"wrist_base_target", "hand_orientation_target"}},
        {"drive", {"forward_backward", "left_right"}}
};
std::vector<int> motor_packet_ids = {
  ID_MOTOR_UNIT_MODE_SEL,
  ID_MOTOR_UNIT_PID_P_SET,
  ID_MOTOR_UNIT_PID_I_SET,
  ID_MOTOR_UNIT_PID_D_SET,
  ID_MOTOR_UNIT_PID_POS_TGT_SET,
  ID_MOTOR_UNIT_PWM_DIR_SET
};
std::vector<int> motor_packet_lengths = {
  2, 5, 5, 5, 5, 5
};

bool ParseMotorPacket(json &message);
bool SendCANPacketToMotor(json &message, int key_idx, uint16_t CAN_ID);
bool ParseIKPacket(json &message);
bool ParseDrivePacket(json &message);

int getIndex(const std::vector<std::string> &arr, std::string &value)
{
  for (int i = 0; i < arr.size(); i++) {
    if (arr[i] == value) {
      return i;
    }
  }
  return -1;
}

bool sendError(std::string const &msg)
{
  json error_message = {};
  error_message["status"] = "error";
  error_message["msg"] = msg;
  sendBaseStationPacket(error_message.dump());
  return false;
}


bool ParseBaseStationPacket(char const* buffer)
{
  std::cout << "Message from base station: " << buffer << std::endl;
  json parsed_message;
  try
  {
    parsed_message = json::parse(buffer);
  }
  catch (json::parse_error)
  {
    return sendError("Parse error");
  }
  std::string type;
  try
  {
    type = parsed_message["type"];
  }
  catch (json::type_error)
  {
    return sendError("Could not find message type");
  }
  std::cout << "Message type: " << type << std::endl;
  bool success;
  if (type == "ik") {
    success = ParseIKPacket(parsed_message);
  }
  else if (type == "drive") {
    success = ParseDrivePacket(parsed_message);
  }
  else if (type == "motor") {
    success = ParseMotorPacket(parsed_message);
  }
  if (success)
  {
    json response = {{"status", "ok"}};
    sendBaseStationPacket(response.dump());
  }
  return success;
}

bool ParseIKPacket(json &message) {
  double ELBOW_LENGTH = Constants::ELBOW_LENGTH;
  double SHOULDER_LENGTH = Constants::SHOULDER_LENGTH;
  for (int key_idx = 0; key_idx < possible_keys["ik"].size(); key_idx++) {
    std::string key = possible_keys["ik"][key_idx];
    if (message[key] != nullptr) {
      if (message[key] == "wrist_base_target") {
        double x = message[key][0];
        double y = message[key][1];
        double z = message[key][2];
        double base_angle = atan(y/x); //TODO Check with electronics
        json base_packet = {{"type", "motor"}, {"motor", "DEVICE_SERIAL_MOTOR_BASE"}, {"mode", "PID"}, {"PID target", base_angle}};

        if (!ParseMotorPacket(base_packet))
        {
          return false;
        }

        double forward = sqrt(x*x + y*y);
        double height = z;

        double crossSection = sqrt(height*height + forward*forward);
        double shoulderAngle = 0;
        double shoulderAngleA = atan(height/forward);
        double elbowAngle = acos((crossSection*crossSection
          - ELBOW_LENGTH*ELBOW_LENGTH - SHOULDER_LENGTH*SHOULDER_LENGTH)
          /(-2*SHOULDER_LENGTH*ELBOW_LENGTH));
              double shoulderAngleB = asin(sin(elbowAngle)*ELBOW_LENGTH/crossSection);
        if (forward == 0)
        {
          shoulderAngle = M_PI/2 - shoulderAngleB;
        }
        else
        {
          shoulderAngle = M_PI - (shoulderAngleA + shoulderAngleB);
        }
        json elbow_packet = {{"type", "motor"}, {"motor", "DEVICE_SERIAL_MOTOR_ELBOW"}, {"mode", "PID"}, {"PID target", elbowAngle}};	
        json shoulder_packet = {{"type", "motor"}, {"motor", "DEVICE_SERIAL_MOTOR_SHOULDER"}, {"mode", "PID"}, {"PID target", shoulderAngle}};
        if (!ParseMotorPacket(elbow_packet))
        {
          return false;
        }
        if (!ParseMotorPacket(shoulder_packet))
        {
          return false;
        }
      }

      //TODO Add functionality for other key.

    }
  }

  return true;
}

bool ParseMotorPacket(json &message)
{
  std::string motor = message["motor"];
  int motor_serial = getIndex(motor_group, motor);
  if (motor_serial < 0)
  {
    return sendError("Unrecognized motor " + motor);
  }
  std::cout << "Parsing motor packet for motor " << motor << std::endl;
  Globals::motor_status[motor]["motor"] = motor;
  std::cout << "Original status: " << Globals::motor_status[motor] << std::endl;


  for (int key_idx = 0; key_idx < possible_keys["motor"].size(); key_idx++) {
    std::string key = possible_keys["motor"][key_idx];
    if (message[key] != nullptr && message[key] != Globals::motor_status[motor][key]) {
      std::cout << "Updating " << key << std::endl;
      uint16_t CAN_ID = ConstructCANID(PACKET_PRIORITY_NORMAL,
                                       DEVICE_GROUP_MOTOR_CONTROL,
                                       motor_serial);
      if (SendCANPacketToMotor(message, key_idx, CAN_ID)) {
        Globals::motor_status[motor][key] = message[key];
      } else {
        return sendError("Failed to send CAN packet to motor " + motor);
      }
    }
  }

  return true;
}

bool ParseDrivePacket(json &message)
{
  double fb, lr;
  try
  {
    fb = message["forward_backward"];
    lr = message["left_right"];
  }
  catch (json::type_error)
  {
    return sendError("Malformatted drive packet");
  }
  if (fb > 1.0 || fb < -1.0 || lr > 1.0 || lr < -1.0)
  {
    return sendError("Drive targets not within bounds +/- 1.0");
  }
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

bool SendCANPacketToMotor(json &message, int key_idx, uint16_t CAN_ID) {
  std::string key = possible_keys["motor"][key_idx];
  int length = motor_packet_lengths[key_idx];
  int packet_id = motor_packet_ids[key_idx];
  uint8_t data[length];
  WritePacketIDOnly(data, packet_id);

  if (key == "mode") {
    std::string mode = message[key];
    if (mode == "PWM") {
      data[1] = MOTOR_UNIT_MODE_PWM;
    } else if (mode == "PID") {
      data[1] = MOTOR_UNIT_MODE_PID;
    } else {
      return sendError("Unrecognized mode " + mode);
    }
  }
  else // key is one of P, I, D, PID target, or PWM target
  {
    uint32_t val = message[key];
    PackIntIntoDataMSBFirst(data, val, 1);
  }

  CANPacket p = ConstructCANPacket(CAN_ID, length, data);
  sendCANPacket(p);
  return true;
}
