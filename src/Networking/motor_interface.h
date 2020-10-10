
#pragma once

#include "json.hpp"
using nlohmann::json;

bool ParseMotorPacket(json &message);
