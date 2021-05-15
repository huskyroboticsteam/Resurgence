
#pragma once

#include <stdint.h>

void InitializeRover(uint8_t arm_mode, bool zero_encoders);
int rover_loop(int argc, char** argv);
