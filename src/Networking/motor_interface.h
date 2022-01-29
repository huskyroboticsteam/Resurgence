
#pragma once

#include <nlohmann/json.hpp>
using nlohmann::json;

bool ParseMotorPacket(json& message);
void incrementArm();
bool motorSupportsPID(int motor_serial);
