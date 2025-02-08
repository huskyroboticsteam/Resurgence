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
	using robot::types::name_to_jointid;
	for (const auto& pair : name_to_jointid) {
		if (pair.second == joint) {
			return std::string(pair.first.data());
		}
	}
	return "<unknown>";
}

std::string to_string(robot::types::motorid_t motor) {
	using robot::types::name_to_motorid;
	for (const auto& pair : name_to_motorid) {
		if (pair.second == motor) {
			return std::string(pair.first.data());
		}
	}
	return "<unknown>";
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
