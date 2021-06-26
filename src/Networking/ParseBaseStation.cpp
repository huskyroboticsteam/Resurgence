
#include "Network.h"
#include "../log.h"
#include "IK.h"
#include "motor_interface.h"
#include "ParseBaseStation.h"
#include <cmath>
#include <tgmath.h>
#include "../simulator/world_interface.h"
#include "../Globals.h"
#include "CANUtils.h"

extern "C"
{
    #include "../HindsightCAN/CANMotorUnit.h"
    #include "../HindsightCAN/CANSerialNumbers.h"
    #include "../HindsightCAN/CANCommon.h"
    #include "../HindsightCAN/CANPacket.h"
}

#include <iostream>

const double DEFAULT_X_VEL = 0.5; // m/s
const double DEFAULT_TH_VEL = 1.0; // rad/s

using nlohmann::json;

bool ParseDrivePacket(json &message);
bool ParseEmergencyStop(json &message);

bool sendError(std::string const &msg)
{
  json error_message = {};
  error_message["status"] = "error";
  error_message["msg"] = msg;
  sendBaseStationPacket(error_message.dump());
  log(LOG_ERROR, "%s\n", error_message.dump().c_str());
  return false;
}

bool ParseBaseStationPacket(char const* buffer)
{
  log(LOG_DEBUG, "Message from base station: %s\n", buffer);
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
  log(LOG_DEBUG, "Message type: %s\n", type.c_str());

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
  else if (type == "autonomous") {
    Globals::AUTONOMOUS = !Globals::AUTONOMOUS;
    success = setCmdVel(0,0);
    log(LOG_INFO, "Set autonomous to %d\n", Globals::AUTONOMOUS);
  } else if (type != "estop") {
    return sendError("Unrecognized message type '" +  type + "'");
  }

  if (success)
  {
    //For some reason sending these packets sometimes hangs. Maybe the string is too long?
    //json response = {{"status", "ok"}, {"data", Globals::status_data}};
    json response = {{"status", "ok"}};
    sendBaseStationPacket(response.dump());
  }
  return success;
}

bool ParseEmergencyStop(json &message) {
  CANPacket p;
  AssembleGroupBroadcastingEmergencyStopPacket(&p,
          DEVICE_GROUP_MOTOR_CONTROL, ESTOP_ERR_GENERAL);
  sendCANPacket(p);
  // For some reason, the above broadcast doesn't seem to be received by the boards.
  // To address this, we also send individual e-stop packets.
  for (uint8_t serial = 1; serial < 12; serial++) {
      AssembleEmergencyStopPacket(&p,
          DEVICE_GROUP_MOTOR_CONTROL, serial, ESTOP_ERR_GENERAL);
      sendCANPacket(p);
  }
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
  return setCmdVel(lr * DEFAULT_TH_VEL, fb * DEFAULT_X_VEL);
}
