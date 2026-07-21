#pragma once

#include "world.h"

namespace robot {

class RealWorldInterface : WorldInterface {

void emergencyStop() override {
    WorldInterface::emergencyStop();
    // CAN here
}

};

}