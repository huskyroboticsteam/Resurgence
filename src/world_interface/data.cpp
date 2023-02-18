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
		case jointid_t::wrist:
			return "wrist";
		case jointid_t::drill_arm:
			return "drillArm";
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
