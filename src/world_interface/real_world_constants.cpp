#include "real_world_constants.h"

using robot::types::motorid_t;

namespace robot {

extern const std::set<motorid_t> pidMotors = {
	motorid_t::armBase, motorid_t::shoulder,		 motorid_t::elbow,
	motorid_t::forearm, motorid_t::differentialLeft, motorid_t::differentialRight};

// TODO: replace with real serial numbers
extern const std::map<motorid_t, can::deviceserial_t> motorSerialIDMap = {
	{motorid_t::frontLeftWheel, 0},
	{motorid_t::frontRightWheel, 1},
	{motorid_t::rearLeftwheel, 2},
	{motorid_t::rearRightWheel, 3},
	{motorid_t::armBase, 4},
	{motorid_t::shoulder, 5},
	{motorid_t::elbow, 6},
	{motorid_t::forearm, 7},
	{motorid_t::differentialRight, 8},
	{motorid_t::differentialLeft, 9},
	{motorid_t::hand, 10}};

// TODO: tune pid
extern const std::map<motorid_t, pidcoef_t> motorPIDMap = {
	{motorid_t::armBase, {1000, 50, 10000}},	 {motorid_t::shoulder, {100, 0, 1000}},
	{motorid_t::elbow, {500, 50, 10000}},		 {motorid_t::forearm, {1000, 0, 0}},
	{motorid_t::differentialLeft, {1000, 0, 0}}, {motorid_t::differentialRight, {1000, 0, 0}}};

// TODO: verify encoder inversions
extern const std::map<motorid_t, bool> motorEncInvMap = {
	{motorid_t::armBase, false},
	{motorid_t::shoulder, false},
	{motorid_t::elbow, true},
	{motorid_t::forearm, false},
	{motorid_t::differentialLeft, false},
	{motorid_t::differentialRight, false}};

// TODO: measure/verify this
extern const std::map<robot::types::motorid_t, uint32_t> motorPulsesPerJointRevMap = {
	{motorid_t::armBase, 17 * 1000},  {motorid_t::armBase, 20 * 1000},
	{motorid_t::armBase, 36 * 1000},  {motorid_t::armBase, 360 * 1000},
	{motorid_t::armBase, 360 * 1000}, {motorid_t::armBase, 360 * 1000}};

} // namespace robot
