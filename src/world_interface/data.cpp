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

constexpr float PotentiometerParams::scale() const {
	return (static_cast<int16_t>(adc_hi) - static_cast<int16_t>(adc_lo))/(mdeg_hi - mdeg_lo);
}
} // namespace robot::types
