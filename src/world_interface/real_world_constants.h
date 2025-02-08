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
constexpr auto encMotors = frozen::make_unordered_map<motorid_t, encparams_t>({
	{motorid_t::shoulder,
		{.isInverted = true,
		.ppjr = 4590 * 1024 * 4,
		.limitSwitchLow = Constants::arm::JOINT_LIMITS.at(robot::types::motorid_t::shoulder).first,
		.limitSwitchHigh = Constants::arm::JOINT_LIMITS.at(robot::types::motorid_t::shoulder).second,
		.zeroCalibrationPower = 0.4}}
});
// clang-format on

// TODO: find appropriate bounds
constexpr auto potMotors = frozen::make_unordered_map<motorid_t, potparams_t>({
	{motorid_t::forearm,
	 {.adc_lo = 1208, .mdeg_lo = -180 * 1000, .adc_hi = 841, .mdeg_hi = 180 * 1000}},
	{motorid_t::wristDiffLeft,
	 {.adc_lo = 0, .mdeg_lo = -100 * 0, .adc_hi = 0, .mdeg_hi = 100 * 0}},
	{motorid_t::wristDiffRight,
	 {.adc_lo = 0, .mdeg_lo = -100 * 0, .adc_hi = 0, .mdeg_hi = 100 * 0}},
	{motorid_t::frontLeftSwerve,
	 {.adc_lo = 1422, .mdeg_lo = -180 * 1000, .adc_hi = 656, .mdeg_hi = 180 * 1000}},
	{motorid_t::frontRightSwerve,
	 {.adc_lo = 644, .mdeg_lo = -180 * 1000, .adc_hi = 1404, .mdeg_hi = 180 * 1000}},
	{motorid_t::rearLeftSwerve,
	 {.adc_lo = 1260, .mdeg_lo = -180 * 1000, .adc_hi = 476, .mdeg_hi = 180 * 1000}},
	{motorid_t::rearRightSwerve,
	 {.adc_lo = 1538, .mdeg_lo = -180 * 1000, .adc_hi = 767, .mdeg_hi = 180 * 1000}},
});

/** @brief A mapping of motorids to their corresponding serial number. */
constexpr auto motorSerialIDMap = frozen::make_unordered_map<motorid_t, can::deviceserial_t>(
	{{motorid_t::frontLeftWheel, DEVICE_SERIAL_MOTOR_CHASSIS_FL},
	 {motorid_t::frontRightWheel, DEVICE_SERIAL_MOTOR_CHASSIS_FR},
	 {motorid_t::rearLeftWheel, DEVICE_SERIAL_MOTOR_CHASSIS_BL},
	 {motorid_t::rearRightWheel, DEVICE_SERIAL_MOTOR_CHASSIS_BR},
	 {motorid_t::frontLeftSwerve, DEVICE_SERIAL_MOTOR_CHASSIS_FL_SW},
	 {motorid_t::frontRightSwerve, DEVICE_SERIAL_MOTOR_CHASSIS_FR_SW},
	 {motorid_t::rearLeftSwerve, DEVICE_SERIAL_MOTOR_CHASSIS_BL_SW},
	 {motorid_t::rearRightSwerve, DEVICE_SERIAL_MOTOR_CHASSIS_BR_SW},
	 {motorid_t::armBase, DEVICE_SERIAL_MOTOR_BASE},
	 {motorid_t::shoulder, DEVICE_SERIAL_MOTOR_SHOULDER},
	 {motorid_t::elbow, DEVICE_SERIAL_MOTOR_ELBOW},
	 {motorid_t::forearm, DEVICE_SERIAL_MOTOR_FOREARM},
	 {motorid_t::wristDiffLeft, DEVICE_SERIAL_MOTOR_WRIST_DIFF_LEFT},
	 {motorid_t::wristDiffRight, DEVICE_SERIAL_MOTOR_WRIST_DIFF_RIGHT},
	 {motorid_t::hand, DEVICE_SERIAL_MOTOR_HAND},
	 {motorid_t::activeSuspension, DEVICE_SERIAL_LINEAR_ACTUATOR},
	 {motorid_t::drillActuator, DEVICE_SERIAL_DRILL_ARM_MOTOR},
	 {motorid_t::drillMotor, DEVICE_SERIAL_DRILL_MOTOR},
	 {motorid_t::scienceStepper, DEVICE_SERIAL_SCIENCE_STEPPER}});

/** @brief A mapping of PID controlled motors to their pid coefficients. */
constexpr auto motorPIDMap = frozen::make_unordered_map<motorid_t, pidcoef_t>(
	{{motorid_t::shoulder, {70, 0, 0}},
	 {motorid_t::frontLeftSwerve, {2, 0, 0}},
	 {motorid_t::frontRightSwerve, {2, 0, 0}},
	 {motorid_t::rearLeftSwerve, {2, 0, 0}},
	 {motorid_t::rearRightSwerve, {2, 0, 0}}});

/**
 * @brief A mapping of motorids to power scale factors when commanded with positive power.
 * Negative values mean that the motor is inverted.
 */
constexpr auto positive_pwm_scales =
	frozen::make_unordered_map<motorid_t, double>({{motorid_t::armBase, -0.25},
												   {motorid_t::shoulder, -1},
												   {motorid_t::elbow, -1},
												   {motorid_t::forearm, -0.2},
												   {motorid_t::wristDiffLeft, -0.1},
												   {motorid_t::wristDiffRight, 0.1},
												   {motorid_t::frontLeftWheel, 0.7},
												   {motorid_t::frontRightWheel, 0.7},
												   {motorid_t::rearLeftWheel, 0.7},
												   {motorid_t::rearRightWheel, 0.7},
												   {motorid_t::frontLeftSwerve, 1.0},
												   {motorid_t::frontRightSwerve, 1.0},
												   {motorid_t::rearLeftSwerve, 1.0},
												   {motorid_t::rearRightSwerve, 1.0},
												   {motorid_t::hand, -0.75},
												   {motorid_t::activeSuspension, -0.5},
												   {motorid_t::drillActuator, -0.5},
												   {motorid_t::drillMotor, 1.0},
												   {motorid_t::scienceStepper, 0.7}});
/**
 * @brief A mapping of motorids to power scale factors when commanded with negative power.
 * Negative values mean that the motor is inverted.
 */
constexpr auto negative_pwm_scales =
	frozen::make_unordered_map<motorid_t, double>({{motorid_t::armBase, -0.25},
												   {motorid_t::shoulder, -1},
												   {motorid_t::elbow, -1},
												   {motorid_t::forearm, -0.2},
												   {motorid_t::wristDiffLeft, -0.1},
												   {motorid_t::wristDiffRight, 0.1},
												   {motorid_t::frontLeftWheel, 0.7},
												   {motorid_t::frontRightWheel, 0.7},
												   {motorid_t::rearLeftWheel, 0.7},
												   {motorid_t::rearRightWheel, 0.7},
												   {motorid_t::frontLeftSwerve, 1.0},
												   {motorid_t::frontRightSwerve, 1.0},
												   {motorid_t::rearLeftSwerve, 1.0},
												   {motorid_t::rearRightSwerve, 1.0},
												   {motorid_t::hand, -0.75},
												   {motorid_t::activeSuspension, -0.5},
												   {motorid_t::drillActuator, -0.5},
												   {motorid_t::drillMotor, 1.0},
												   {motorid_t::scienceStepper, 0.7}});

} // namespace robot
