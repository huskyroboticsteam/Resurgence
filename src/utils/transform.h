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

} // namespace util
