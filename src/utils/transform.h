#pragma once

#include "../navtypes.h"

#include <Eigen/Geometry>

namespace util {

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
double quatToHeading(const Eigen::Quaterniond& quat);

/**
 * @brief Convert euler angles to a quaternion representation.
 *
 * @param rpy The euler angles to convert.
 * @return Eigen::Quaterniond The same orientation represented as a quaternion.
 */
Eigen::Quaterniond eulerAnglesToQuat(const navtypes::eulerangles_t& rpy);

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
 * This rigid transform takes points from the robot frame to
 * the global frame.
 *
 * @param pose The current robot pose.
 * @return navtypes::transform_t The rigid transform denoting the pose of the robot.
 */
navtypes::transform_t toTransform(const navtypes::pose_t& pose);

/**
 * @brief Convert a pose to a rigid transform.
 *
 * Calculate the rigid transform associated with the given pose.
 * This rigid transform takes points from the robot frame to
 * the global frame.
 *
 * @param x The x position
 * @param y The y position
 * @param theta The heading
 * @return navtypes::transform_t The rigid transform denoting the pose of the robot.
 */
navtypes::transform_t toTransform(double x, double y, double theta);

} // namespace util
