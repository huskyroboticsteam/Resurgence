#pragma once

#include "navtypes.h"

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

navtypes::points_t transformReadings(const navtypes::points_t& ps,
									 const navtypes::transform_t& tf);
navtypes::trajectory_t transformTraj(const navtypes::trajectory_t& traj,
									 const navtypes::transform_t& tf);

/* Whether the robot in location given by `tf` collides with anything. */
bool collides(const navtypes::transform_t& tf, const navtypes::obstacles_t& obss);
bool collides(const navtypes::transform_t& tf, const navtypes::points_t& lms, double radius);

/* Returns the smallest number t in the unit interval [0,1] such that
 * the point r0 + t(r1-r0) lies on an obstacle boundary. If there is no such point,
 * returns t=2.0. */
double obstacleIntersection(const navtypes::point_t& r0, const navtypes::point_t& r1,
							const navtypes::obstacles_t& obss);

/* Left-multiplying a (map-frame) point_t by this matrix will give the
 * corresponding point_t in the frame that was rotated counterclockwise
 * by theta, then shifted by (x,y) along the new (x,y)-axes.
 *
 * For example, for (x, y, theta) = (1, 0, pi/2), the origin frame
 *
 *   . .
 *   > .
 *
 * becomes
 *
 *   ^ .
 *   . .
 */
navtypes::transform_t toTransformRotateFirst(double x, double y, double theta);

// Given a transform, gives the robot pose that would have that transform.
// That is, the x,y location and angle theta of the robot such that left-multiplying a
// world-frame landmark location by `trf` will give you the robot-frame landmark location.
//
// NOTE: THIS IS AN INVERSE OF toTransform, NOT toTransformRotateFirst.
// The parameters of toTransformRotateFirst are not a robot pose
// (rotation and translation operations do not commute).
navtypes::pose_t toPose(const navtypes::transform_t& trf, double prev_theta);
double nearestHeadingBranch(double theta, double prev_theta);

navtypes::transform_t toTransform(const navtypes::pose_t& pose);

// One sample from a standard normal distribution.
// The thread_id is used to choose which random number generator to use.
// This is important when trying to rerun a particular random seed; each thread
// needs its own dedicated sequence of random numbers.
double stdn(int thread_id);
// Returns the random seed used for the stdn() function
long getNormalSeed();

} // namespace util
