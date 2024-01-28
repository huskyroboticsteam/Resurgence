#include "core.h"

namespace util {

bool almostEqual(double a, double b, double threshold) {
	return std::abs(a - b) < threshold;
}

template <>
std::string to_string<bool>(const bool& val) {
	return val ? "true" : "false";
}

frozen::string freezeStr(const std::string& str) {
	return frozen::string(str.c_str(), str.size());
}

RAIIHelper::RAIIHelper(const std::function<void()>& f) : f(f) {}

RAIIHelper::RAIIHelper(RAIIHelper&& other) : f(std::move(other.f)) {
	other.f = {};
}

RAIIHelper::~RAIIHelper() {
	if (f) {
		f();
	}
}

} // namespace util
