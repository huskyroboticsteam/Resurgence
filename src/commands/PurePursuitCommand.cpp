#include "PurePursuitCommand.h"

#include <loguru.hpp>
#include "../utils/transform.h"

#include <cmath>
#include <algorithm>

using navtypes::point_t;
using navtypes::points_t;
using navtypes::pose_t;

namespace {
static double wrapAngle(double angErr) {
	return std::atan2(std::sin(angErr), std::cos(angErr));
}

static double sgn(double num) {
	if (num < 0) {
		return -1.0;
	} else {
		return 1.0;
	}
}
} // namespace

namespace commands {

PurePursuitCommand::PurePursuitCommand(const points_t& waypoints)
	: _pose(pose_t::Zero()) {
	interpolatePoints(waypoints);
	// LOG_F(INFO, "path size: %ld", _path.size());
}

void PurePursuitCommand::setState(const pose_t& pose) {
	LOG_F(INFO, "pose: %f %f, path[1]: %f %f, path[2]: %f %f",
		_pose[0], _pose[1],
		_path[1][0], _path[1][1],
		_path[2][0], _path[2][1]);
	this->_pose = pose;
	this->_set_state_called_before_output = true;
}

command_t PurePursuitCommand::getOutput() {
	// LOG_F(INFO, "entering get ouput");
	LOG_F(INFO, "curr idx: %d, path_size: %ld", _curr_idx, _path.size());

    if (!this->_set_state_called_before_output) {
		LOG_F(WARNING, "PurePursuitCommand: getOutput() called before setState() call!");
		return;
	}

	double distToGoal = (_pose.head<2>() - _path.back().head<2>()).norm();

	double driveVel = _drive_vel;
	if (distToGoal <= _slow_thresh && _curr_idx >= _path.size() - 2) {
		driveVel *= distToGoal / _slow_thresh;
	}
	if (distToGoal <= _done_thresh && _curr_idx >= _path.size() - 2) {
        return {.thetaVel = 0.0, .xVel = 0.0};
    }

	// LOG_F(INFO, "updating index now");
	updateCurrentIndex();
	point_t relIntersect;
	if (_curr_idx >= _path.size() - 1) {
		relIntersect = util::toTransform(_pose) * _path.back();
	} else {
		// lineToCircleIntersection already returns robot frame coords
		relIntersect = lineToCircleIntersection(_path[_curr_idx], _path[_curr_idx + 1]);
	}

	LOG_F(INFO, "Intersect: x=%f y=%f", relIntersect[0], relIntersect[1]);

	double curvature = (2 * relIntersect[1]) 
						/ (relIntersect[0]*relIntersect[0] + relIntersect[1]*relIntersect[1]);

	double thetaVel = driveVel * curvature; // curvature times drive vel = theta vel

	// LOG_F(INFO, "%f, %f", thetaVel, driveVel);
	return {.thetaVel = thetaVel, .xVel = driveVel};
}

point_t PurePursuitCommand::lineToCircleIntersection(point_t& p1, point_t& p2) {
	// Transform argument points to robot frame (center = (0,0))
	point_t p1Robot = util::toTransform(_pose) * p1;
	point_t p2Robot = util::toTransform(_pose) * p2;
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
	double disc = _lookahead_dist*_lookahead_dist * dr2 - D*D;

	LOG_F(INFO, "DISC only: %f", disc);

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

		// These are rearranged equations of form: point - p1 = t * (p2 - p1)
		double t1 = ((xSol1 - x1) * dx + (ySol1 - y1) * dy) / dr2;
		double t2 = ((xSol2 - x1) * dx + (ySol2 - y1) * dy) / dr2;

		// Leave room for some error for floating point
		bool valid1 = (t1 >= 0 && t1 <= 1);
		bool valid2 = (t2 >= 0 && t2 <= 1);

		LOG_F(INFO, "valid1: %d, valid2: %d, disc: %f", valid1, valid2, disc);

		if (valid1 && !valid2) return sol1;
		if (valid2 && !valid1) return sol2;
		if (valid1 && valid2) return (t1 > t2 ? sol1 : sol2);
	}

	return p2Robot;
}

void PurePursuitCommand::interpolatePoints(const points_t& waypoints) {
	if (waypoints.empty()) {
		return;
	}
	_path.push_back(waypoints[0]);
	point_t lastPathPoint = waypoints[0];
	int numPts = 1;
	double accumulatedDist = 0;

	for (int i = 0; i < waypoints.size() - 1; i++) {
		point_t p1 = waypoints[i];
		point_t p2 = waypoints[i + 1];
		double segmentLen = (p2.head<2>() - p1.head<2>()).norm();

		while (accumulatedDist + segmentLen >= numPts * _dist_between_points) {
			// Find new points by parameterizing slope between p1 and p2
			double t = (numPts * _dist_between_points - accumulatedDist) / segmentLen;
			point_t newPt = {p1[0] + t * (p2[0] - p1[0]), p1[1] + t * (p2[1] - p1[1]), 1};
			_path.push_back(newPt);
			lastPathPoint = newPt;
			numPts++;
		}

		accumulatedDist += segmentLen;
	}
	if (dist(_path.back(), waypoints.back()) >= 0.01) {
		_path.push_back(waypoints.back());	
	}
}

bool PurePursuitCommand::isDone() {
	double distance = (_pose.topRows<2>() - _path.back().topRows<2>()).norm();
	// LOG_F(INFO, "dist to goal %f", distance);
	if (distance <= _done_thresh && _curr_idx >= _path.size() - 2) {
		LOG_F(INFO, "distance from goal: %lf", distance);
		LOG_F(INFO, "done +1");
		_done_count++;
	} else {
		// If we've left the done threshold, reset done count
		_done_count = 0;
	}

	// Must be in done thresh for at least 100ms (5 iterations of control loop)
	// to be considered "done"
	return _done_count >= 5;
}

void PurePursuitCommand::updateCurrentIndex() {
	double distToNext;
	while (_curr_idx < _path.size() - 2) {
		distToNext = (_pose.topRows<2>() - _path[_curr_idx + 1].topRows<2>()).norm();
		if (distToNext < _lookahead_dist) {
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
