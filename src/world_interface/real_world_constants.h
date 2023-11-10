#pragma once

#include "../CAN/CANUtils.h"
#include "../Constants.h"
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
constexpr auto pidMotors =
	frozen::make_unordered_set<motorid_t>({motorid_t::shoulder, motorid_t::elbow});

/**
 * @brief Represents parameters defining a potentiometer scale.
 *
 * Contains two joint angles in millidegrees and their associated potentiometer ADC values;
 * this defines a linear scale from potentiometer ADC value to joint angle that can be sent to
 * the motor boards for position control and feedback.
 */
struct potparams_t {
	/** The "low" point on the ADC scale. */
	uint16_t adc_lo;
	/** The "low" point on the joint rotation scale. */
	int32_t mdeg_lo;
	/** The "high" point on the ADC scale. */
	uint16_t adc_hi;
	/** The "high" point on the joint rotation scale. */
	int32_t mdeg_hi;
};

struct encparams_t {
	/** Whether the encoder motor is inverted. */
	bool isInverted;
	/** Encoder pulses count per joint revolution */
	int ppjr;
	/** Limit switch low, in millidegrees */
	int limitSwitchLow;
	/** Limit switch high, in millidegrees */
	int limitSwitchHigh;
	/** Power value set during limit switch calibration */
	double zeroCalibrationPower;
};

// clang-format off
constexpr auto encMotors = frozen::make_unordered_map<motorid_t, encparams_t>(
	{{motorid_t::shoulder,
		{.isInverted = true,
		.ppjr = 4590 * 1024 * 4,
		.limitSwitchLow = Constants::arm::JOINT_LIMITS.at(robot::types::motorid_t::shoulder).first,
		.limitSwitchHigh = Constants::arm::JOINT_LIMITS.at(robot::types::motorid_t::shoulder).second,
		.zeroCalibrationPower = 0.4,
		.stdPos = 2*(M_PI/3)}},
	{motorid_t::elbow,
		{.isInverted = false,
		.ppjr = 1620 * 1024 * 4,
		.limitSwitchLow = Constants::arm::JOINT_LIMITS.at(robot::types::motorid_t::elbow).first,
		.limitSwitchHigh = Constants::arm::JOINT_LIMITS.at(robot::types::motorid_t::elbow).second,
		.zeroCalibrationPower = -0.15,
		.stdPos = 0.5*(M_PI)}}});
// clang-format on

// TODO: find appropriate bounds
constexpr auto potMotors = frozen::make_unordered_map<motorid_t, potparams_t>(
	{{motorid_t::armBase,
	  {.adc_lo = 123, .mdeg_lo = -200 * 1000, .adc_hi = 456, .mdeg_hi = 200 * 1000}},
	 {motorid_t::forearm,
	  {.adc_lo = 1208, .mdeg_lo = -180 * 1000, .adc_hi = 841, .mdeg_hi = 180 * 1000}},
	 {motorid_t::wrist,
	  {.adc_lo = 123, .mdeg_lo = -100 * 1000, .adc_hi = 456, .mdeg_hi = 100 * 1000}},
	 {motorid_t::activeSuspension,
	  {.adc_lo = 251, .mdeg_lo = -19 * 1000, .adc_hi = 1645, .mdeg_hi = 31 * 1000}}});

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
	 {motorid_t::wrist, DEVICE_SERIAL_MOTOR_WRIST},
	 {motorid_t::hand, DEVICE_SERIAL_MOTOR_HAND},
	 {motorid_t::activeSuspension, DEVICE_SERIAL_LINEAR_ACTUATOR}});

/** @brief A mapping of PID controlled motors to their pid coefficients. */
constexpr auto motorPIDMap = frozen::make_unordered_map<motorid_t, pidcoef_t>(
	{{motorid_t::shoulder, {70, 0, 0}}, {motorid_t::elbow, {15, 7, -2}}});

/**
 * @brief A mapping of motorids to power scale factors when commanded with positive power.
 * Negative values mean that the motor is inverted.
 */
constexpr auto positive_pwm_scales =
	frozen::make_unordered_map<motorid_t, double>({{motorid_t::armBase, -0.75},
												   {motorid_t::shoulder, 1},
												   {motorid_t::elbow, 1},
												   {motorid_t::forearm, -0.65},
												   {motorid_t::wrist, 0.6},
												   {motorid_t::frontLeftWheel, 0.7},
												   {motorid_t::frontRightWheel, -0.7},
												   {motorid_t::rearLeftWheel, 0.7},
												   {motorid_t::rearRightWheel, 0.7},
												   {motorid_t::hand, -0.75},
												   {motorid_t::activeSuspension, -0.5}});
/**
 * @brief A mapping of motorids to power scale factors when commanded with negative power.
 * Negative values mean that the motor is inverted.
 */
constexpr auto negative_pwm_scales =
	frozen::make_unordered_map<motorid_t, double>({{motorid_t::armBase, -0.75},
												   {motorid_t::shoulder, 1},
												   {motorid_t::elbow, 1},
												   {motorid_t::forearm, -0.65},
												   {motorid_t::wrist, 0.6},
												   {motorid_t::frontLeftWheel, 0.7},
												   {motorid_t::frontRightWheel, -0.7},
												   {motorid_t::rearLeftWheel, 0.7},
												   {motorid_t::rearRightWheel, 0.7},
												   {motorid_t::hand, -0.75},
												   {motorid_t::activeSuspension, -0.5}});

} // namespace robot
