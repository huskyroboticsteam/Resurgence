
#pragma once

#include <stdint.h>

void setMotorMode(uint8_t serial, uint8_t mode);
int rover_loop(int argc, char** argv);

constexpr double CONTROL_HZ = 10.0;
