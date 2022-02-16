#pragma once

#include <chrono>
#include <string>
#include <time.h>

#include <Eigen/Geometry>
#include <sys/time.h>

namespace util {
bool almostEqual(double a, double b, double threshold = 1e-6);

/**
 * @brief Extract the heading from a quaternion.
 *
 * Given a quaternion defined by out coordinate system, extract the CCW heading, in radians.
 *
 * @param qw The w component of the quaternion.
 * @param qx The x component of the quaternion.
 * @param qy The y component of the quaternion.
 * @param qz The z component of the quaternion.
 * @return double The CCW heading, in radians.
 */
double quatToHeading(double qw, double qx, double qy, double qz);

/**
 * @brief Extract the heading from a quaternion.
 *
 * Given a quaternion defined by out coordinate system, extract the CCW heading, in radians.
 *
 * @param quat The quaternion to extract the heading from. This does not need to be normalized.
 * @return double The CCW heading, in radians.
 */
double quatToHeading(Eigen::Quaterniond quat);

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
