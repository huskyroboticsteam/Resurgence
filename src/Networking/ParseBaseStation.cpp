
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
std::map<std::string, std::vector<std::string>> possible_keys = {{"motor", {}}, {"ik", {}}, {"drive", {}}}
possible_keys["motor"] = {"mode", "P", "I", "D", "PID target", "PWM target"};
possible_keys["ik"] = {"wrist_base_target", "hand_orientation_target"};
possible_keys["drive"] = {"forward_backward", "left_right"};

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

void ParseMotorPacket(json &message);
bool SendCANPacketToMotor(json &message, int key_idx, uint16_t CAN_ID);

int getIndex(const std::vector<std::string> &arr, std::string &value)
{
  for (int i = 0; i < arr.size(); i++) {
    if (arr[i] == value) {
      return i;
    }
  }
  return -1;
}

void ParseBaseStationPacket(char* buffer)
{
  std::cout << "Message from base station: " << buffer << std::endl;
  json parsed_message = json::parse(buffer);
  // TODO proper input validation. Sometimes the rover crashes due to malformatted json
  std::string type = parsed_message["type"];
  std::cout << "Message type: " << type << std::endl;
  // TODO implement inverse kinematics
  if (type == "ik") {
    ParseIKPacket(parsed_message);
  if (type == "motor") {
    ParseMotorPacket(parsed_message);
  }
  sendBaseStationPacket(Globals::status_data.dump());
}

void ParseIKPacket(json &message) {
  for (int key_idx = 0; key_idx < possible_keys["ik"].size(); key_idx++) {
    std::string key = possible_keys["ik"][key_idx];
    if (message[key] != nullptr) {
      if (message[key] == "wrist_base_target") {
        double x = message[key][0];
        double y = message[key][1];
	double z = message[key][2];
        //TODO Send arctan(y/x) to base joint
	
	double forward = sqrt(x*x + y*y);
	double height = z;

	double crossSection = sqrt(height*height + forward*forward);
	double shoulderAngleA = atan(height/forward);
	double elbowAngle = acos((crossSection*crossSection - ELBOW_LENGTH*ELBOW_LENGTH - SHOULDER_LENGTH*SHOULDER_LENGTH)/(-2*SHOULDER_LENGTH*ELBOW_LENGTH));
        double shoulderAngleB = asin(sin(elbowAngle)*ELBOW_LENGTH/crossSection);
	if (forward == 0)
	{
	  double shoulderAngle = M_PI/2 - shoulderAngleB;
	}
	else
        {
		shoulderAngle = M_PI - (shoulderAngleA + shoulderAngleB);
	}
	//TODO Send shoulderAngle and elbowAngle to shoulder and elbow joints
      }   
    }
  }
}

void ParseMotorPacket(json &message)
{
  std::string motor = message["motor"];
  int motor_serial = getIndex(motor_group, motor);
  if (motor_serial < 0)
  {
    std::cout << "Unrecognized motor " << motor << std::endl;
    return;
  }
  std::cout << "Parsing motor packet for motor " << motor << std::endl;
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
      }
    }
  }

  std::cout << "Final status: " << motor_status[motor] << std::endl << std::endl;
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
