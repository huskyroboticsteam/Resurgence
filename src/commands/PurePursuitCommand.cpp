#include "PurePursuitCommand.h"

#include "../Constants.h"

#include <loguru.hpp>
#include "../utils/transform.h"

#include <cmath>
#include <algorithm>
#include <fstream>  // temporary for testing!!
#include <iostream>
#include "../CAN/CAN.h"

using navtypes::point_t;
using navtypes::points_t;
using navtypes::pose_t;

namespace c_c = Constants::commands;

namespace {
	
double wrapAngle(const double angErr) {
	return std::atan2(std::sin(angErr), std::cos(angErr));
}

double sgn(const double num) {
	if (num < 0) {
		return -1.0;
	} else {
		return 1.0;
	}
}

double dist(const Eigen::Vector3d& one, const Eigen::Vector3d& two) {
	return (one.head<2>() - two.head<2>()).norm();
}

} // namespace (anonymous)

namespace commands {

PurePursuitCommand::PurePursuitCommand(const points_t waypoints)
	: _pose(pose_t::Zero()) {
	file.open("logpath.csv", std::ios::trunc);
	LOG_F(INFO, "pathlog file is open: %d\n", file.is_open());
	interpolatePoints(waypoints);
}

PurePursuitCommand::~PurePursuitCommand() {
	file.close();
}

void PurePursuitCommand::setState(const pose_t& pose) {
	this->_pose = pose;
	this->_set_state_called_before_output = true;
}

command_t PurePursuitCommand::getOutput() {
	if (_path.empty()) {
        LOG_F(ERROR, "getOutput called with empty path");
        return {.thetaVel = 0.0, .xVel = 0.0};
    }

    if (!this->_set_state_called_before_output) {
		LOG_F(WARNING, "PurePursuitCommand: getOutput() called before setState() call!");
	}

	double distToGoal = dist(_pose, _path.back());
	double driveVel = c_c::DRIVE_VEL;
	// Close to goal
	// _curr_idx check is especially for circle paths where start == end.
	// Checking makes sure that rover has completed circle and is not just
	// starting it.
	if (distToGoal <= c_c::SLOW_THRESH && _curr_idx >= _path.size() - 2) {
		driveVel *= distToGoal / c_c::SLOW_THRESH;
	}
	// At goal
	if (distToGoal <= c_c::DONE_THRESH && _curr_idx >= _path.size() - 2) {
        return {.thetaVel = 0.0, .xVel = 0.0};
    }

	// Try to update index
	updateCurrentIndex();
	point_t relIntersect;
	if (_curr_idx >= _path.size() - 1) {
		// Convert to robot frame
		relIntersect = util::toTransform(_pose) * _path.back();
	} else {
		// lineToCircleIntersection already returns robot frame coords
		relIntersect = lineToCircleIntersection(_path[_curr_idx], _path[_curr_idx + 1]);
	}

	double curvature = (2 * relIntersect[1]) 
						/ (relIntersect[0]*relIntersect[0] + relIntersect[1]*relIntersect[1]);

	double thetaVel = driveVel * curvature; // curvature times drive vel = theta vel

	LOG_F(INFO, "%f, %f", thetaVel, driveVel);
	return {.thetaVel = thetaVel, .xVel = driveVel};
}

point_t PurePursuitCommand::lineToCircleIntersection(const point_t& p1, const point_t& p2) {
	// Transform argument points to robot frame (center = (0,0))
	point_t p1Robot = util::toTransform(_pose) * p1;
	point_t p2Robot = util::toTransform(_pose) * p2;

	point_t zero = {0,0,0};
	if (dist(zero, p1Robot) > dist(zero, p2Robot)) {
		LOG_F(ERROR, "p1 is further than p2!");
	}
	
	double x1 = p1Robot[0];
	double y1 = p1Robot[1];
	double x2 = p2Robot[0];
	double y2 = p2Robot[1];

	double dx = x2 - x1;
	double dy = y2 - y1;
	double dr2 = pow(dx, 2) + pow(dy, 2);

	double D = x1 * y2 - x2 * y1;

	// If discriminant is 0, there is 1 intersections
	//                 is > 0, there are 2 intersections
	//                 is < 0, there are no intersections
	double disc = c_c::LOOKAHEAD_DIST*c_c::LOOKAHEAD_DIST * dr2 - D*D;

	if (disc >= 0) {
		// There exists at least one intersection.
		// From now on, if only one intersection exists (disc == 0), sol1 == sol2
		double xSol1 = (D * dy + sgn(dy) * dx * sqrt(disc)) / dr2;
		double xSol2 = (D * dy - sgn(dy) * dx * sqrt(disc)) / dr2;
		double ySol1 = (-D * dx + abs(dy) * sqrt(disc)) / dr2;
		double ySol2 = (-D * dx - abs(dy) * sqrt(disc)) / dr2;

		point_t sol1 = { xSol1, ySol1, 1 };
		point_t sol2 = { xSol2, ySol2, 1 };
	

		// Parameterize line that passes through p1 and p2 such that t = 0 is p1
		// and t = 1 is p2.
		// Find the t-values of each found intersection point on the line.
		// Higher t-value indicates further along on line.

		// These are rearranged equations of form: point - p1 = t * (p2 - p1) s.t.
		// t = 0 is p1 and t = 1 is p2
		double t1 = ((xSol1 - x1) * dx + (ySol1 - y1) * dy) / dr2;
		double t2 = ((xSol2 - x1) * dx + (ySol2 - y1) * dy) / dr2;

		bool valid1 = (t1 >= 0 && t1 <= 1);
		bool valid2 = (t2 >= 0 && t2 <= 1);

		if (valid1 && !valid2) return sol1;
		if (valid2 && !valid1) return sol2;
		if (valid1 && valid2) return (t1 > t2 ? sol1 : sol2);
	}
	return p2Robot;
}

void PurePursuitCommand::interpolatePoints(const points_t& waypoints) {
	if (waypoints.size() < 2) {
		_path = waypoints;
		return;
	}

	
	std::vector<double> cumulativeDist(waypoints.size(), 0.0);

	// Measure distance accumulated at each waypoint, e.g.
	// [ 0.0, 1.2, 4.5, 5.0 ], where each value represents
	// the accumulated distance at waypoint index i.
	cumulativeDist[0] = 0.0;
	for (int i = 1; i < waypoints.size(); i++) {
		cumulativeDist[i] = cumulativeDist[i-1] + dist(waypoints[i-1], waypoints[i]);
	}
	double totalLen = cumulativeDist.back();
	
	// Find how many points we need (at least 2 for start & end)
	int numPts = std::max(2, (int) std::ceil(totalLen / c_c::DIST_BETWEEN_POINTS));

	for (int i = 0; i < numPts; i++) {
		// We want numPts - 1 segments (to get numPts points)
		// targetDist gives us the desired acummulated dist for the ith point
		double targetDist = totalLen * i / (numPts - 1);

		// Goal: Find which waypoints point i will lie between
		// Finds first waypoint cumulative distance >= to our target distance
		auto it = std::lower_bound(cumulativeDist.begin(), cumulativeDist.end(), targetDist);
		// Subtracts one to get start of our desired segment, with clamping to prevent illegal idx
		int idx = std::clamp((int) std::distance(cumulativeDist.begin(), it) - 1,
												 0,
												 (int) waypoints.size() - 2);

		// Interpolate new point on segment
		double segmentLen = cumulativeDist[idx+1] - cumulativeDist[idx];
		double t = segmentLen == 0 ? 0.0 : (targetDist - cumulativeDist[idx]) / segmentLen;
		_path.push_back({
					waypoints[idx][0] + t * (waypoints[idx+1][0] - waypoints[idx][0]),
					waypoints[idx][1] + t * (waypoints[idx+1][1] - waypoints[idx][1]),
					1
				});
	}
}

bool PurePursuitCommand::isDone() {
	if (_path.empty()) {
        LOG_F(ERROR, "PurePursuitCommand path is empty");
        return true;
    }

	double distance = dist(_pose, _path.back());
	if (distance <= _done_thresh && _curr_idx >= static_cast<int>(_path.size()) - 2) {
		_done_count++;
		LOG_F(INFO, "%d. distance from goal: %lf", _done_count, distance);
	} else {
		// If we've left the done threshold, reset done count
		_done_count = 0;
	}

	// Must be in done thresh for at least 100ms (5 iterations of control loop)
	// to be considered "done"
	if (_done_count >= 20) {
		can::setLED(can::led_t::green);
		return true;
	}
}

void PurePursuitCommand::updateCurrentIndex() {
	if (_path.size() < 2) {
		return;
	}
	double distToNext;
	while (_curr_idx < _path.size() - 2) {
		distToNext = dist(_pose, _path[_curr_idx + 1]);
		if (distToNext < c_c::LOOKAHEAD_DIST) {
			_curr_idx++;
		} else {
			break;
		}
	}
}

void PurePursuitCommand::reset() {
    _curr_idx = 0;
    _done_count = 0;
    _set_state_called_before_output = false;
}

} // namespace commands
