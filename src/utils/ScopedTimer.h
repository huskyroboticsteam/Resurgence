#pragma once

#include <chrono>
#include <string>

namespace util {

/**
 * @brief A utility class that helps with timing.
 *
 * Measures the elapsed time from the moment this
 * class is instantiated to the moment that it's destructed.
 */
class ScopedTimer {
public:
	/**
	 * @brief Create a scoped timer with the specified name.
	 *
	 * The elapsed time will be printed to stdout when destructed.
	 *
	 * @param name The name of this timer.
	 */
	ScopedTimer(std::string name);

	/**
	 * @brief Create an unnamed scoped timer.
	 *
	 * Nothing will be printed by this timer.
	 */
	ScopedTimer();

	~ScopedTimer();

	/**
	 * @brief Gets the elapsed time measured by this timer.
	 *
	 * @return The elapsed time, in microseconds.
	 */
	std::chrono::microseconds elapsedTime() const;

private:
	std::chrono::time_point<std::chrono::high_resolution_clock> startTime;
	std::string name;
};

} // namespace util
