#pragma once

#include "../CAN/CANUtils.h"
#include "data.h"

#include <chrono>
#include <cstdint>
#include <unordered_map>
#include <unordered_set>

#include <frozen/unordered_map.h>
#include <frozen/unordered_set.h>

namespace robot {

using types::motorid_t;

/** @brief A struct containing a set of PID coefficients. */
struct pidcoef_t {
	int32_t kP, kI, kD;
};

// TODO: measure to see optimal telemetry period
/** @brief The default telemetry period for motors. */
constexpr std::chrono::milliseconds TELEM_PERIOD(50);

/** @brief The set of motors that are PID controlled. */
constexpr auto pidMotors = frozen::make_unordered_set<motorid_t>(
	{motorid_t::armBase, motorid_t::shoulder, motorid_t::elbow, motorid_t::forearm,
	 motorid_t::differentialLeft, motorid_t::differentialRight});

/** @brief A mapping of motorids to their corresponding serial number. */
constexpr auto motorSerialIDMap = frozen::make_unordered_map<motorid_t, can::deviceserial_t>(
	{{motorid_t::frontLeftWheel, DEVICE_SERIAL_MOTOR_CHASSIS_FL},
	 {motorid_t::frontRightWheel, DEVICE_SERIAL_MOTOR_CHASSIS_FR},
	 {motorid_t::rearLeftWheel, DEVICE_SERIAL_MOTOR_CHASSIS_BL},
	 {motorid_t::rearRightWheel, DEVICE_SERIAL_MOTOR_CHASSIS_BR},
	 {motorid_t::armBase, DEVICE_SERIAL_MOTOR_BASE},
	 {motorid_t::shoulder, DEVICE_SERIAL_MOTOR_SHOULDER},
	 {motorid_t::elbow, DEVICE_SERIAL_MOTOR_ELBOW},
	 {motorid_t::forearm, DEVICE_SERIAL_MOTOR_FOREARM},
	 {motorid_t::differentialRight, DEVICE_SERIAL_MOTOR_DIFF_WRIST_R},
	 {motorid_t::differentialLeft, DEVICE_SERIAL_MOTOR_DIFF_WRIST_L},
	 {motorid_t::hand, DEVICE_SERIAL_MOTOR_HAND}});

// TODO: tune pid
/** @brief A mapping of PID controlled motors to their pid coefficients. */
constexpr auto motorPIDMap = frozen::make_unordered_map<motorid_t, pidcoef_t>(
	{{motorid_t::armBase, {1000, 50, 10000}},
	 {motorid_t::shoulder, {100, 0, 1000}},
	 {motorid_t::elbow, {500, 50, 10000}},
	 {motorid_t::forearm, {1000, 0, 0}},
	 {motorid_t::differentialLeft, {1000, 0, 0}},
	 {motorid_t::differentialRight, {1000, 0, 0}}});

// TODO: verify encoder inversions
/**
 * @brief A map that represents whether a motor's encoder is inverted.
 *
 * If a value is true, that motor's encoder should be inverted.
 * If a motor is not in this map, its encoder should not be inverted, if it has one.
 */
constexpr auto motorEncInvMap =
	frozen::make_unordered_map<motorid_t, bool>({{motorid_t::armBase, false},
												 {motorid_t::shoulder, false},
												 {motorid_t::elbow, true},
												 {motorid_t::forearm, false},
												 {motorid_t::differentialLeft, false},
												 {motorid_t::differentialRight, false}});

// TODO: measure/verify this
/** @brief A mapping of motorids to the number of pulses per joint revolution. */
constexpr auto motorPulsesPerJointRevMap = frozen::make_unordered_map<motorid_t, uint32_t>(
	{{motorid_t::armBase, 17 * 1000},
	 {motorid_t::shoulder, 20 * 1000},
	 {motorid_t::elbow, 36 * 1000},
	 {motorid_t::forearm, 360 * 1000},
	 {motorid_t::differentialLeft, 360 * 1000},
	 {motorid_t::differentialRight, 360 * 1000}});

/**
 * @brief A mapping of motorids to power scale factors when commanded with positive power.
 * Negative values mean that the motor is inverted.
*/
constexpr auto positive_pwm_scales =
	frozen::make_unordered_map<motorid_t, double>({{motorid_t::armBase, 0.1831},
												   {motorid_t::shoulder, -1},
												   {motorid_t::elbow, 0.75},
												   {motorid_t::forearm, 0.1},
												   {motorid_t::differentialLeft, -0.5},
												   {motorid_t::differentialRight, 0.5},
												   {motorid_t::frontLeftWheel, -0.5},
												   {motorid_t::frontRightWheel, 0.5},
												   {motorid_t::rearLeftWheel, -0.5},
												   {motorid_t::rearRightWheel, -0.5},
												   {motorid_t::hand, 0.2289}});
/**
 * @brief A mapping of motorids to power scale factors when commanded with negative power.
 * Negative values mean that the motor is inverted.
*/
constexpr auto negative_pwm_scales =
	frozen::make_unordered_map<motorid_t, double>({{motorid_t::armBase, 0.1831},
												   {motorid_t::shoulder, -1},
												   {motorid_t::elbow, 0.75},
												   {motorid_t::forearm, 0.1},
												   {motorid_t::differentialLeft, -0.5},
												   {motorid_t::differentialRight, 0.5},
                                                   {motorid_t::frontLeftWheel, -0.5},
												   {motorid_t::frontRightWheel, 0.5},
												   {motorid_t::rearLeftWheel, -0.5},
												   {motorid_t::rearRightWheel, -0.5},
												   {motorid_t::hand, 0.2289}});

} // namespace robot
