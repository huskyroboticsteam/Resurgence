#pragma once

#include "navtypes.h"

#include <chrono>
#include <string>
#include <frozen/string.h>
#include <unordered_map>
#include <unordered_set>

#include <Eigen/Geometry>

/**
 * @namespace util
 * @brief A collection of utility functions and classes with common use-cases.
 */
namespace util {

/**
 * @brief Check if two numbers are approximately equal.
 *
 * @param a The first number.
 * @param b The second number.
 * @param threshold If @c |a-b|&ge;threshold then they are not approximately equal.
 * @return true iff @c |a-b|<threshold.
 */
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

/**
 * @brief Convert a set of points to a different reference frame.
 *
 * Applies @p tf^-1 to every element of @p ps and returns the transformed points.
 *
 * @param ps The points to convert between reference frames.
 * @param tf The transform whose inverse will be applied to the points.
 * @return Points such that the the i-th point is equivalent to @c tf^-1*ps[i].
 *
 * @note The inverse of the given transform is applied to every point,
 * not the transform itself.
 */
navtypes::points_t transformReadings(const navtypes::points_t& ps,
									 const navtypes::transform_t& tf);

/**
 * @brief Convert a trajectory to a different reference frame.
 *
 * Applies @p tf on the right for every element of @p traj and returns
 * the transformed trajectory.
 *
 * @param traj The trajectory to transform.
 * @param tf The transform to apply on the right to every element of @p traj.
 * @return The transformed trajectory, such that the i-th element is @c traj[i]*tf.
 */
navtypes::trajectory_t transformTraj(const navtypes::trajectory_t& traj,
									 const navtypes::transform_t& tf);

/**
 * @brief Check for intersection between a pose and a point cloud.
 *
 * Checks if any point of @p lms is within @p radius distance of the robot.
 *
 * @param tf The robot transform.
 * @param lms The points to check against for intersection.
 * @param radius The minimum distance from the robot to a
 * point to not count as an intersection.
 * @return If any point of @p lms is within @p radius distance of the robot.
 */
bool collides(const navtypes::transform_t& tf, const navtypes::points_t& lms, double radius);

/**
 * @brief Creates a rigid transform that takes a point from the map-frame
 * to a frame that was rotated CCW by theta and shifted (x,y) along the new axes.
 *
 * This can also be viewed as rotating a map-frame point CW by theta and
 * then shifting by (-x,-y) along the map axes.
 *
 * @param x The x-coordinate in map-space of the origin of the new frame.
 * @param y The y-coordinate in map-space of the origin of the new frame.
 * @param theta The CCW angle between the new frame and the old frame.
 * @return A rigid transform that changes frames as described.
 */
navtypes::transform_t toTransformRotateFirst(double x, double y, double theta);

/**
 * @brief Given a transform, gives a robot pose that would have that transform.
 *
 * That is, the x,y location and angle theta of the robot such that left-multiplying
 * a map-frame coordinate by `trf` will give you the robot-frame coordinate.
 *
 * @param trf The rigid transform representing a robot position.
 * This transform would take points from the map frame to the robot frame.
 * @param prev_theta Since heading is modular, the returned heading value will be
 * chosen to be within 2&pi; radians of this parameter.
 * @return The robot pose.
 *
 * @warning THIS IS AN INVERSE OF util::toTransform, NOT util::toTransformRotateFirst.
 * The parameters of the latter are not a robot pose
 * (rotation and translation operations do not commute).
 */
navtypes::pose_t toPose(const navtypes::transform_t& trf, double prev_theta);

/**
 * @brief Get an equivalent heading that is within &pi; radians of another heading.
 *
 * @param theta The heading to get the equivalent heading for.
 * @param prev_theta The heading that the returned heading is in the neighborhood of.
 * @return A heading equivalent to @p theta that is within &pi; radians of @p prev_theta.
 */
double closestHeading(double theta, double prev_theta);

/**
 * @brief Convert a pose to a rigid transform.
 *
 * Calculate the rigid transform associated with the given pose.
 * This rigid transform takes points from the global frame to
 * the robot frame.
 *
 * @param pose The current robot pose.
 * @return navtypes::transform_t The rigid transform denoting the pose of the robot.
 */
navtypes::transform_t toTransform(const navtypes::pose_t& pose);

/**
 * @brief Sample from the standard normal distribution.
 *
 * The thread_id is used to choose which random number generator to use.
 * This is important when trying to rerun a particular random seed;
 * each thread needs its own dedicated sequence of random numbers.
 *
 * @param thread_id Either 0 or 1, denoting which random number generator use.
 * @return double
 */
double stdn(int thread_id);

/**
 * @brief Get the random seed used for util::stdn().
 *
 * @return The random seed.
 */
long getNormalSeed();

/**
 * @brief Get the keys of the given map.
 *
 * @param An unordered map.
 * @return The keys of the given map, as an unordered set.
 */
template <typename K, typename V>
std::unordered_set<K> keySet(const std::unordered_map<K, V>& input) {
	std::unordered_set<K> output;
	std::transform(input.begin(), input.end(), std::inserter(output, output.end()),
				   [](auto pair) { return pair.first; });
	return output;
}

/**
 * @brief Convert the given value to a string.
 *
 * This method is necessary because we cannot extend the std namespace; having our own method
 * allows us to extend it with template specializations whenever we want, and have it "fall
 * back" to using the std version when no specialization is available.
 *
 * @param val The value to convert to string.
 * @tparam The type of the value to convert to string.
 * @return A string representation of that value, as a std::string. The exact representation of
 * the value is up to the implementation.
 */
template <typename T>
std::string to_string(const T& val) {
	return std::to_string(val);
}

/**
 * @brief Convert the given std::string to a frozen::string.
 */
frozen::string freezeStr(const std::string& str);

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
	return std::chrono::duration_cast<std::chrono::duration<double, std::chrono::seconds::period>>(dur).count();
}
} // namespace util
