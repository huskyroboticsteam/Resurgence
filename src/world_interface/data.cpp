#include "data.h"

namespace robot::types {

LimitSwitchData::LimitSwitchData(unsigned long long data) : data(data) {}

bool LimitSwitchData::isOpen(size_t idx) {
	return !data.test(idx);
}

bool LimitSwitchData::isClosed(size_t idx) {
	return data.test(idx);
}

bool LimitSwitchData::isAnyClosed() {
	return data.any();
}

bool LimitSwitchData::isAnyOpen() {
	return !data.all();
}

std::bitset<N_LIMIT_SWITCH> LimitSwitchData::diff(const LimitSwitchData& other) {
	return data ^ other.data;
}
} // namespace robot::types

namespace util {

std::string to_string(robot::types::jointid_t joint) {
	using robot::types::jointid_t;
	switch (joint) {
		case jointid_t::armBase:
			return "armBase";
		case jointid_t::shoulder:
			return "shoulder";
		case jointid_t::elbow:
			return "elbow";
		case jointid_t::forearm:
			return "forearm";
		case jointid_t::hand:
			return "hand";
		case jointid_t::wristPitch:
			return "wristPitch";
		case jointid_t::wristRoll:
			return "wristRoll";
		case jointid_t::activeSuspension:
			return "activeSuspension";
		case jointid_t::ikUp:
			return "ikUp";
		case jointid_t::ikForward:
			return "ikForward";
		case jointid_t::fourBarLinkage1:
			return "fourBarLinkage1";
		case jointid_t::fourBarLinkage2:
			return "fourBarLinkage2";
		default:
			// should never happen
			return "<unknown>";
	}
}

std::string to_string(const robot::types::CameraID& id) {
	return id;
}

std::string to_string(robot::types::mountedperipheral_t peripheral) {
	using robot::types::mountedperipheral_t;

	switch (peripheral) {
		case mountedperipheral_t::none:
			return "none";

		case mountedperipheral_t::arm:
			return "arm";

		case mountedperipheral_t::armServo:
			return "armServo";

		case mountedperipheral_t::lidar:
			return "lidar";

		case mountedperipheral_t::scienceStation:
			return "scienceStation";

		default:
			// should never happen
			return "<unknown>";
	}
}

} // namespace util
