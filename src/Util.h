#pragma once

#include <chrono>
#include <string>
#include <time.h>

#include <sys/time.h>

namespace util {
bool almostEqual(double a, double b, double threshold = 1e-6);

/**
 * A utility class that helps with timing. Measures the elapsed time from the moment
 * this class is instantiated to the moment that the scope ends.
 */
class ScopedTimer {
public:
	/**
	 * Create a scoped timer with the specified name. The elapsed time will be printed
	 * to stdout at the end of the scope.
	 * @param name The name of this timer.
	 */
	ScopedTimer(std::string name);

	/**
	 * Create an unnamed scoped timer. Nothing will be printed by this timer.
	 */
	ScopedTimer();
	~ScopedTimer();

	/**
	 * Gets the elapsed time measured by this scoped timer.
	 * @return
	 */
	std::chrono::microseconds elapsedTime() const;

private:
	std::chrono::time_point<std::chrono::high_resolution_clock> startTime;
	std::string name;
};
} // namespace util

long getElapsedUsecs(const struct timeval& tp_start, const struct timeval& tp_end);
long getElapsedUsecs(const struct timeval& tp_start);
