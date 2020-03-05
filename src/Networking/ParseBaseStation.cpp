
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

json motor_status = {};

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

int getIndex(const std::vector<std::string> &arr, std::string &value)
{
  for (int i = 0; i < arr.size(); i++) {
    if (arr[i] == value) {
      return i;
    }
  }
  return -1;
}

bool ParseBaseStationPacket(char const* buffer)
{
  std::cout << "Message from base station: " << buffer << std::endl;
  json parsed_message;
  try
  {
    parsed_message = json::parse(buffer);
  }
  catch (json::parse_error) {
    std::cout << "Parse error\n";
    return false;
  }
  std::string type = parsed_message["type"];
  std::cout << "Message type: " << type << std::endl;
  if (type == "ik") {
    return ParseIKPacket(parsed_message);
  }  
  if (type == "motor") {
    return ParseMotorPacket(parsed_message);
  }
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
          std::cout << "Failed to send CAN packet to base motor.\n";
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
          std::cout << "Failed to send CAN packet to elbow motor.\n";
	  return false;
	}
        if (!ParseMotorPacket(shoulder_packet))
	{
          std::cout << "Failed to send CAN packet to shoulder motor.\n";
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
    std::cout << "Unrecognized motor " << motor << std::endl;
    return false;
  }
  std::cout << "Parsing motor packeast for motor " << motor << std::endl;
  motor_status[motor]["motor"] = motor;
  std::cout << "Original status: " << motor_status[motor] << std::endl;


  for (int key_idx = 0; key_idx < possible_keys["motor"].size(); key_idx++) {
    std::string key = possible_keys["motor"][key_idx];
    if (message[key] != nullptr && message[key] != motor_status[motor][key]) {
      std::cout << "Updating " << key << std::endl;
      uint16_t CAN_ID = ConstructCANID(PACKET_PRIORITY_NORMAL,
                                       DEVICE_GROUP_MOTOR_CONTROL,
                                       motor_serial);
      if (SendCANPacketToMotor(message, key_idx, CAN_ID)) {
        motor_status[motor][key] = message[key];
      } else {
        std::cout << "Failed to send CAN packet.\n";
        return false;
      }
    }
  }

  sendBaseStationPacket(motor_status[motor].dump());
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
      std::cout << "Error: unrecognized mode " << mode << std::endl;
      return false;
    }
  }
  else // key is one of P, I, D, PID target, or PWM target
  {
    uint32_t val = message[key];
    PackIntIntoDataMSBFirst(data, val, 1);
  }

  CANPacket p = ConstructCANPacket(CAN_ID, length, data);
  SendCANPacket(p);
  return true;
}
