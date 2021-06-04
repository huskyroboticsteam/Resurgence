
#pragma once

#include "json.hpp"
using nlohmann::json;

bool ParseMotorPacket(json &message);
void incrementArm();
bool motorSupportsPID(int motor_serial);
