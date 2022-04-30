#pragma once

#include "../CAN/CANUtils.h"
#include "data.h"

#include <chrono>
#include <cstdint>
#include <unordered_map>
#include <unordered_set>

namespace robot {

/** @brief A struct containing a set of PID coefficients. */
struct pidcoef_t {
	int32_t kP, kI, kD;
};

// TODO: measure to see optimal telemetry period
/** @brief The default telemetry period for motors. */
constexpr std::chrono::milliseconds TELEM_PERIOD(50);

/** @brief The set of motors that are PID controlled. */
extern const std::unordered_set<robot::types::motorid_t> pidMotors;

/** @brief A mapping of motorids to their corresponding serial number. */
extern const std::unordered_map<robot::types::motorid_t, can::deviceserial_t> motorSerialIDMap;

/** @brief A mapping of PID controlled motors to their pid coefficients. */
extern const std::unordered_map<robot::types::motorid_t, pidcoef_t> motorPIDMap;

/**
 * @brief A map that represents whether a motor's encoder is inverted.
 *
 * If a value is true, that motor's encoder should be inverted.
 * If a motor is not in this map, it's encoder should not be inverted, if it has one.
 */
extern const std::unordered_map<robot::types::motorid_t, bool> motorEncInvMap;

/** @brief A mapping of motorids to the number of pulses per joint revolution. */
extern const std::unordered_map<robot::types::motorid_t, uint32_t> motorPulsesPerJointRevMap;

} // namespace robot
