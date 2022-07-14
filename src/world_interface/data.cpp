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
std::string to_string(const robot::types::CameraID& id) {
	return id;
}
}
