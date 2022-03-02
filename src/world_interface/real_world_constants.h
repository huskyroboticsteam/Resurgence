#pragma once

#include "../CAN/CANUtils.h"
#include "data.h"

#include <cstdint>
#include <map>
#include <set>

#include <chrono>

namespace robot {

struct pidcoef_t {
	int32_t kP, kI, kD;
};

// TODO: measure to see optimal telemetry period
constexpr std::chrono::milliseconds TELEM_PERIOD(50);

extern const std::set<robot::types::motorid_t> pidMotors;

extern const std::map<robot::types::motorid_t, can::deviceserial_t> motorSerialIDMap;
extern const std::map<robot::types::motorid_t, pidcoef_t> motorPIDMap;
extern const std::map<robot::types::motorid_t, bool> motorEncInvMap;
extern const std::map < robot::types::motorid_t, uint32_t> motorPulsesPerJointRevMap;

} // namespace robot
