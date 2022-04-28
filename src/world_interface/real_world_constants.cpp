#include "real_world_constants.h"

using robot::types::motorid_t;

namespace robot {

extern const std::unordered_set<motorid_t> pidMotors = {
	motorid_t::armBase, motorid_t::shoulder,		 motorid_t::elbow,
	motorid_t::forearm, motorid_t::differentialLeft, motorid_t::differentialRight};

extern const std::unordered_map<motorid_t, can::deviceserial_t> motorSerialIDMap = {
	{motorid_t::frontLeftWheel, DEVICE_SERIAL_MOTOR_CHASSIS_FL},
	{motorid_t::frontRightWheel, DEVICE_SERIAL_MOTOR_CHASSIS_FR},
	{motorid_t::rearLeftwheel, DEVICE_SERIAL_MOTOR_CHASSIS_BL},
	{motorid_t::rearRightWheel, DEVICE_SERIAL_MOTOR_CHASSIS_BR},
	{motorid_t::armBase, DEVICE_SERIAL_MOTOR_BASE},
	{motorid_t::shoulder, DEVICE_SERIAL_MOTOR_SHOULDER},
	{motorid_t::elbow, DEVICE_SERIAL_MOTOR_ELBOW},
	{motorid_t::forearm, DEVICE_SERIAL_MOTOR_FOREARM},
	{motorid_t::differentialRight, DEVICE_SERIAL_MOTOR_DIFF_WRIST_R},
	{motorid_t::differentialLeft, DEVICE_SERIAL_MOTOR_DIFF_WRIST_L},
	{motorid_t::hand, DEVICE_SERIAL_MOTOR_HAND}};

// TODO: tune pid
extern const std::unordered_map<motorid_t, pidcoef_t> motorPIDMap = {
	{motorid_t::armBase, {1000, 50, 10000}},	 {motorid_t::shoulder, {100, 0, 1000}},
	{motorid_t::elbow, {500, 50, 10000}},		 {motorid_t::forearm, {1000, 0, 0}},
	{motorid_t::differentialLeft, {1000, 0, 0}}, {motorid_t::differentialRight, {1000, 0, 0}}};

// TODO: verify encoder inversions
extern const std::unordered_map<motorid_t, bool> motorEncInvMap = {
	{motorid_t::armBase, false},
	{motorid_t::shoulder, false},
	{motorid_t::elbow, true},
	{motorid_t::forearm, false},
	{motorid_t::differentialLeft, false},
	{motorid_t::differentialRight, false}};

// TODO: measure/verify this
extern const std::unordered_map<robot::types::motorid_t, uint32_t> motorPulsesPerJointRevMap = {
	{motorid_t::armBase, 17 * 1000},  {motorid_t::armBase, 20 * 1000},
	{motorid_t::armBase, 36 * 1000},  {motorid_t::armBase, 360 * 1000},
	{motorid_t::armBase, 360 * 1000}, {motorid_t::armBase, 360 * 1000}};

} // namespace robot
