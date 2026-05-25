#pragma once

// new
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

using types::boardid_t;

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
constexpr auto encMotors = frozen::make_unordered_map<boardid_t, encparams_t>({
	{boardid_t::shoulder,
		{.isInverted = true,
		.ppjr = 4590 * 1024 * 4,
		.limitSwitchLow = Constants::arm::JOINT_LIMITS.at(robot::types::boardid_t::shoulder).first,
		.limitSwitchHigh = Constants::arm::JOINT_LIMITS.at(robot::types::boardid_t::shoulder).second,
		.zeroCalibrationPower = 0.4}},
});

constexpr double FOURBAR_GEAR_RATIO = 71.71875;
// clang-format on

// TODO: find appropriate bounds
constexpr auto potMotors = frozen::make_unordered_map<boardid_t, potparams_t>({
	{boardid_t::forearm,
	 {.adc_lo = 1208, .mdeg_lo = -180 * 1000, .adc_hi = 841, .mdeg_hi = 180 * 1000}},
	{boardid_t::wristDiffLeft,
	 {.adc_lo = 0, .mdeg_lo = -100 * 0, .adc_hi = 0, .mdeg_hi = 100 * 0}},
	{boardid_t::wristDiffRight,
	 {.adc_lo = 0, .mdeg_lo = -100 * 0, .adc_hi = 0, .mdeg_hi = 100 * 0}}
});

/** @brief A mapping of board UUID (boardid_t) to their corresponding uuid. */
constexpr auto boardUUIDMap = frozen::make_unordered_map<boardid_t, CANDevice_t>(
	{// BLDC Motors - Use BLDC commands
	 /*
	 {boardid_t::leftTread, CANDevice_t{0, 1, 0, CAN_UUID_BLDC_FRONT_TIRE_LEFT}},
	 {boardid_t::rightTread, CANDevice_t{0, 1, 0, CAN_UUID_BLDC_FRONT_TIRE_RIGHT}},
	 */
	 {boardid_t::frontTireLeft, CANDevice_t{0, 1, 0, CAN_UUID_BLDC_FRONT_TIRE_LEFT}},
	 {boardid_t::frontTireRight, CANDevice_t{0, 1, 0, CAN_UUID_BLDC_FRONT_TIRE_RIGHT}},
	 {boardid_t::rearTireLeft, CANDevice_t{0, 1, 0, CAN_UUID_BLDC_REAR_TIRE_LEFT}},
	 {boardid_t::rearTireRight, CANDevice_t{0, 1, 0, CAN_UUID_BLDC_REAR_TIRE_RIGHT}},
	 {boardid_t::armBase, CANDevice_t{0, 1, 0, CAN_UUID_BLDC_BASE}},
	 {boardid_t::shoulder, CANDevice_t{0, 1, 0, CAN_UUID_BLDC_SHOULDER}},
	 {boardid_t::elbow, CANDevice_t{0, 1, 0, CAN_UUID_BLDC_ELBOW}},
	 {boardid_t::forearm, CANDevice_t{0, 1, 0, CAN_UUID_BLDC_FOREARM}},
	 {boardid_t::wristDiffLeft, CANDevice_t{0, 1, 0, CAN_UUID_BLDC_WRIST_LEFT}},
	 {boardid_t::wristDiffRight, CANDevice_t{0, 1, 0, CAN_UUID_BLDC_WRIST_RIGHT}},
	
	 // Telemetry (0x50)
	 {boardid_t::telemetry, CANDevice_t{1, 0, 0, CAN_UUID_TELEMETRY}},
	 // Hand (0x60)
	 {boardid_t::hand, CANDevice_t{0, 1, 0, CAN_UUID_HAND}},
	 // DEBUG (0x70, 0x71)
	 {boardid_t::debug1, CANDevice_t{1, 0, 0, CAN_UUID_DEBUG1}},
	 {boardid_t::debug2, CANDevice_t{1, 0, 0, CAN_UUID_DEBUG2}}
	});

constexpr auto UUIDBoardMap = frozen::make_unordered_map<CANDeviceUUID_t, boardid_t>(
	{
	 {CAN_UUID_BLDC_FRONT_TIRE_LEFT, boardid_t::frontTireLeft},
	 {CAN_UUID_BLDC_FRONT_TIRE_RIGHT, boardid_t::frontTireRight},
	 {CAN_UUID_BLDC_REAR_TIRE_LEFT, boardid_t::rearTireLeft},
	 {CAN_UUID_BLDC_REAR_TIRE_RIGHT, boardid_t::rearTireRight},
	 {CAN_UUID_BLDC_BASE, boardid_t::armBase},
	 {CAN_UUID_BLDC_SHOULDER, boardid_t::shoulder},
	 {CAN_UUID_BLDC_ELBOW, boardid_t::elbow},
	 {CAN_UUID_BLDC_FOREARM, boardid_t::forearm},
	});

constexpr auto boardInversionMap = frozen::make_unordered_map<boardid_t, int8_t>({
	 {boardid_t::frontTireLeft, -1},
	 {boardid_t::frontTireRight, 1},
	 {boardid_t::rearTireLeft, -1},
	 {boardid_t::rearTireRight, 1},
	 {boardid_t::armBase, 1},
	 {boardid_t::shoulder, 1},
	 {boardid_t::elbow, 1},
	 {boardid_t::forearm, 1},
	 {boardid_t::wristDiffLeft, 1},
	 {boardid_t::wristDiffRight, 1},
	
	 // Telemetry (0x50)
	 {boardid_t::telemetry, 0},
	 // Hand (0x60)
	 {boardid_t::hand, 1},
	 // DEBUG (0x70, 0x71)
	 {boardid_t::debug1, 1},
	 {boardid_t::debug2, 1}
});

constexpr auto proBoards = frozen::make_unordered_set<boardid_t>({
	boardid_t::armBase,
	boardid_t::shoulder,
	boardid_t::elbow,
	boardid_t::debug2,
});

constexpr auto boardBrakeIDMap = frozen::make_unordered_map<boardid_t, uint8_t>({
	{boardid_t::armBase, 5},
	{boardid_t::shoulder, 2},
	{boardid_t::elbow, 1}
});

/** @brief A mapping of PID controlled motors to their pid coefficients. */
constexpr auto motorPIDMap =
	frozen::make_unordered_map<boardid_t, pidcoef_t>({{boardid_t::shoulder, {70, 0, 0}}});

/**
 * @brief A mapping of motorids to power scale factors when commanded with positive power.
 * Negative values mean that the motor is inverted.
 */
constexpr auto positive_pwm_scales =
	frozen::make_unordered_map<boardid_t, double>({{boardid_t::armBase, 1},
												   {boardid_t::shoulder, -1},
												   {boardid_t::elbow, -1},
												   {boardid_t::forearm, -0.01},
												   {boardid_t::wristDiffLeft, -0.1},
												   {boardid_t::wristDiffRight, 0.1},
												   {boardid_t::frontTireLeft, -3},
												   {boardid_t::frontTireRight, 3},
												   {boardid_t::rearTireLeft, -3},
												   {boardid_t::rearTireRight, 3},
												   {boardid_t::hand, -0.75}});
/**
 * @brief A mapping of motorids to power scale factors when commanded with negative power.
 * Negative values mean that the motor is inverted.
 */
constexpr auto negative_pwm_scales =
	frozen::make_unordered_map<boardid_t, double>({{boardid_t::armBase, -1},
												   {boardid_t::shoulder, -1},
												   {boardid_t::elbow, -1},
												   {boardid_t::forearm, -0.01},
												   {boardid_t::wristDiffLeft, -0.1},
												   {boardid_t::wristDiffRight, 0.1},
												   {	boardid_t::frontTireLeft, -3},
												   {boardid_t::frontTireRight, 3},
												   {boardid_t::rearTireLeft, -3},
												   {boardid_t::rearTireRight, 3},
												   {boardid_t::hand, -0.75}});

} // namespace robot
