
#include "Network.h"
#include "log.h"
#include "IK.h"
#include "motor_interface.h"
#include "ParseBaseStation.h"
#include <cmath>
#include <tgmath.h>
#include "../simulator/world_interface.h"
#include "../Globals.h"

#include <iostream>

using nlohmann::json;

bool ParseDrivePacket(json &message);
bool ParseEmergencyStop(json &message);

bool sendError(std::string const &msg)
{
  json error_message = {};
  error_message["status"] = "error";
  error_message["msg"] = msg;
  sendBaseStationPacket(error_message.dump());
  log(LOG_ERROR, error_message.dump());
  return false;
}

bool ParseBaseStationPacket(char const* buffer)
{
  log(LOG_INFO, "Message from base station: " + (std::string) buffer);
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
  log(LOG_DEBUG, "Message type: " + type);

  bool success = false;
  if (type == "estop") {
    success = ParseEmergencyStop(parsed_message);
  } else if (Globals::E_STOP) {
    return sendError("Emergency stop is activated");
  }

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

bool ParseEmergencyStop(json &message) {
  // TODO actually send e-stop packet (packet id 0x30 broadcast)
  bool success = setCmdVel(0,0);
  Globals::E_STOP = true;
  bool release;
  try
  {
    release = message["release"];
  }
  catch (json::type_error)
  {
    return sendError("Malformatted estop packet");
  }

  if (release) {
    Globals::E_STOP = false;
    return success;
  } else {
    return success;
  }
}

bool ParseDrivePacket(json &message) {
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
  // TODO do we need to scale or invert these?
  return setCmdVel(lr, fb);
}
