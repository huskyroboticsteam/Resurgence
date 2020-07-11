#pragma once

#include <vector>

extern const std::vector<std::string> broadcast_group;
extern const std::vector<std::string> reserved_group;
extern const std::vector<std::string> master_group;
extern const std::vector<std::string> power_group;
extern const std::vector<std::string> motor_group;
extern const std::vector<std::string> telemetry_group;
extern const std::vector<std::string> gpio_group;

extern const std::vector<std::vector<std::string>> can_groups;

extern const std::vector<std::string> telem_types;

void ParseCANPacket(CANPacket p);
