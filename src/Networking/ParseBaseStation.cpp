
#include "Network.h"
#include "../Globals.h"
#include "json.hpp"
#include <iostream>

using nlohmann::json;

json motor_status = {};

void ParseMotorPacket(json &message);
void SendCANPacketToMotor(json &message, std::string &key);

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
  std::cout << "Parsing motor packet for motor " << motor << std::endl;
  motor_status[motor]["motor"] = motor;
  std::cout << "Original status: " << motor_status[motor] << std::endl;

  std::vector<std::string> possible_keys = {"mode", "P", "I", "D", "PID target", "PWM target"};

  for (std::string key : possible_keys) {
    if (message[key] != nullptr && message[key] != motor_status[motor][key]) {
      // TODO input validation: e.g. if this is a PID target, make sure message[key] is
      // an integer. Or maybe this could be handled in a subroutine
      motor_status[motor][key] = message[key];
      std::cout << "Updating " << key << std::endl;
      SendCANPacketToMotor(message, key);
    }
  }

  std::cout << "Final status: " << motor_status[motor] << std::endl << std::endl;
}

void SendCANPacketToMotor(json &message, std::string &key) {
  if (key == "mode") {
    std::string mode = message[key];
    if (mode == "PWM") {
      // TODO: Send CAN packet changing the mode to PWM
      // See TestPackets.cpp and CANUtils.cpp and FakeCANBoard.cpp for examples
    } else if (mode == "PID") {
      // TODO: Send CAN packet changing the mode to PID
    } else {
      std::cout << "Error: unrecognized mode " << mode << std::endl;
    }
  }
}
