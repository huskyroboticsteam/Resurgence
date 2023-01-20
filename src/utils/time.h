#pragma once

#include <chrono>

namespace util {

using dseconds = std::chrono::duration<double, std::chrono::seconds::period>;

/**
 * @brief Convert a duration to seconds, as a double.
 * 
 * @tparam Rep The rep of the duration.
 * @tparam Period The period of the duration.
 * @param dur The duration to convert.
 * @return double The length of the duration, in seconds.
 */
template <typename Rep, typename Period>
double durationToSec(std::chrono::duration<Rep, Period> dur) {
	return std::chrono::duration_cast<dseconds>(dur).count();
}

}
