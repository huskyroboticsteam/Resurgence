
#ifndef _PLAN_H_
#define _PLAN_H_

#include <Eigen/Core>

#include "../simulator/utils.h"

// Sequence of controls. Each control is a (x distance, theta distance) pair.
using plan_t = Eigen::Matrix<double, Eigen::Dynamic, 2>;
using action_t = Eigen::Vector2d;

plan_t getPlan(const std::function<bool(double x, double y, double radius)> &, const point_t &, double);
plan_t getPlan(const points_t &lidar_scan, const point_t &goal, double goal_radius);
double planCostFromIndex(plan_t &plan, int idx);

#endif
