
#include "Network.h"
#include "../Globals.h"
#include "json.hpp"
#include "CANUtils.h"
#include "ParseCAN.h"

extern "C"
{
    #include "../HindsightCAN/CANMotorUnit.h"
}

#include <iostream>

using nlohmann::json;

json motor_status = {};

void ParseMotorPacket(json &message);
bool SendCANPacketToMotor(json &message, std::string &key, int motor_serial);

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
  if (type == "motor") {
    ParseMotorPacket(parsed_message);
  }
  sendBaseStationPacket(Globals::status_data.dump());
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

  std::vector<std::string> possible_keys = {"mode", "P", "I", "D", "PID target", "PWM target"};

  for (std::string key : possible_keys) {
    if (message[key] != nullptr && message[key] != motor_status[motor][key]) {
      std::cout << "Updating " << key << std::endl;
      if (SendCANPacketToMotor(message, key, motor_serial)) {
        motor_status[motor][key] = message[key];
      } else {
        std::cout << "Failed to send CAN packet.\n";
      }
    }
  }

  std::cout << "Final status: " << motor_status[motor] << std::endl << std::endl;
}

bool SendCANPacketToMotor(json &message, std::string &key, int motor_serial) {
  if (key == "mode") {
    std::string mode = message[key];
    uint16_t CAN_ID = ConstructCANID(PACKET_PRIORITY_NORMAL,
                                     DEVICE_GROUP_MOTOR_CONTROL,
                                     motor_serial);
    uint8_t length = 0x02;
    uint8_t data[length];
    WritePacketIDOnly(data, ID_MOTOR_UNIT_MODE_SEL);

    if (mode == "PWM") {
      data[1] = MOTOR_UNIT_MODE_PWM;
    } else if (mode == "PID") {
      data[1] = MOTOR_UNIT_MODE_PID;
    } else {
      std::cout << "Error: unrecognized mode " << mode << std::endl;
      return false;
    }
    CANPacket p = ConstructCANPacket(CAN_ID, length, data);
    SendCANPacket(p);
  }

  return true;
}
