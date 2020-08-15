
#include <string>
#include "log.h"
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
	{"motor", {"mode", "P", "I", "D", "PID target", "PWM target"}},
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

  log(LOG_DEBUG, "Parsing motor packet for motor " + motor);
  Globals::motor_status[motor]["motor"] = motor;
  log(LOG_DEBUG, "Original status: " + Globals::motor_status[motor].dump());


  for (int key_idx = 0; key_idx < possible_keys["motor"].size(); key_idx++) {
    std::string key = possible_keys["motor"][key_idx];
    if (message[key] != nullptr && message[key] != Globals::motor_status[motor][key]) {
      log(LOG_DEBUG, "Updating " + key);
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
