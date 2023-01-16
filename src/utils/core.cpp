#include "core.h"

namespace util{

bool almostEqual(double a, double b, double threshold) {
	return std::abs(a - b) < threshold;
}

frozen::string freezeStr(const std::string& str) {
	return frozen::string(str.c_str(), str.size());
}

}