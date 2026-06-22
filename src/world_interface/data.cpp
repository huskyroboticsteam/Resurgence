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

std::string to_string(robot::types::boardid_t board) {
	using robot::types::boardid_t;
	switch (board) {
		case boardid_t::frontTireLeft: return "frontTireLeft";
		case boardid_t::frontTireRight: return "frontTireRight";
		case boardid_t::rearTireLeft: return "rearTireLeft";
		case boardid_t::rearTireRight: return "rearTireRight";
		case boardid_t::armBase: return "armBase";
		case boardid_t::shoulder: return "shoulder";
		case boardid_t::elbow: return "elbow";
		case boardid_t::forearm: return "forearm";
		case boardid_t::wristDiffLeft: return "wristDiffLeft";
		case boardid_t::wristDiffRight: return "wristDiffRight";
		case boardid_t::telemetry: return "telemetry";
		case boardid_t::hand: return "hand";
		case boardid_t::debug1: return "debug1";
		case boardid_t::debug2: return "debug2";
		default: return "<unknown>";
	}
}

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
		case jointid_t::wristPitch:
			return "wristPitch";
		case jointid_t::wristRoll:
			return "wristRoll";
		case jointid_t::hand:
			return "hand";
		case jointid_t::handActuator:
			return "handActuator";
		case jointid_t::laser:
			return "laser";
		default:
			// should never happen
			return "<unknown>";
	}
}

std::string to_string(robot::types::mountedperipheral_t peripheral) {
	using robot::types::mountedperipheral_t;

	switch (peripheral) {
		case mountedperipheral_t::none:
			return "none";

		case mountedperipheral_t::arm:
			return "arm";

		case mountedperipheral_t::science:
			return "science";

		default:
			// should never happen
			return "<unknown>";
	}
}

std::string to_string(robot::types::servoid_t servo) {
	using robot::types::servoid_t;

	switch (servo) {
		case servoid_t::mast:
			return "mast";
		default:
			return "<unknown>";
	}
}

} // namespace util
