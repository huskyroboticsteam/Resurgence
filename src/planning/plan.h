
#ifndef _PLAN_H_
#define _PLAN_H_

#include <Eigen/Core>

#include "../simulator/utils.h"

// Sequence of controls. Each control is a (x distance, theta distance) pair.
using plan_t = Eigen::Matrix<double, Eigen::Dynamic, 2>;
using action_t = Eigen::Vector2d;

/**
 * Finds a path to the given goal point, stopping when at most goal_radius distance away.
 *
 * @param collides A collision predicate, used to determine if the given x and y coords are
 * a at most a given distance away from any point. Users can implement this callback using
 * whatever algorithm they want. Note that the x and y coords will be in the robot's local space,
 * at the time of invoking this method.
 * @param goal The goal point to pathfind towards. This is in robot local space.
 * @param goal_radius The maximum allowable distance to the goal.
 * @return A plan that takes the robot from its current position to the goal point.
 */
plan_t getPlan(const std::function<bool(double x, double y, double radius)> &collides, const point_t &goal, double goal_radius);

/**
 * Finds a path to the given goal point, stopping when at most goal_radius distance away.
 * This is functionally equivalent to the other getPlan() method, but uses a linear scan
 * through the lidar points to check for collisions, instead of something more efficient.
 * For large amounts of points, you should prefer the other getPlan() method.
 *
 * @param lidar_scan A vector of points given by the lidar scan. These should all be in robot local space.
 * @param goal The goal point to pathfind towards. This is in robot local space.
 * @param goal_radius The maximum allowable distance to the goal.
 * @return A plan that takes the robot from its current position to the goal point.
 */
plan_t getPlan(const points_t &lidar_scan, const point_t &goal, double goal_radius);
double planCostFromIndex(plan_t &plan, int idx);

#endif
