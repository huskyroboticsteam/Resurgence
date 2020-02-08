
#include "Network.h"
#include "../Globals.h"
#include "json.hpp"
#include <iostream>

void ParseBaseStationPacket(char* buffer)
{
  std::cout << "Message from base station: " << buffer << std::endl;
  nlohmann::json parsed_message = nlohmann::json::parse(buffer);
  // TODO proper input validation. Here we assume we got a dict with key "speed"
  std::cout << parsed_message["speed"] << std::endl;
  sendBaseStationPacket(Globals::status_data.dump());
}
