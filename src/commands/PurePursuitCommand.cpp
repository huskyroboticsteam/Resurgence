#include "PurePursuitCommand.h"

#include <loguru.hpp>
#include "../utils/transform.h"

#include <algorithm>
#include <cmath>

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

PurePursuitCommand::PurePursuitCommand(const points_t& waypoints, double driveVel,
									   double doneThresh)
	: _pose(pose_t::Zero()), _drive_vel(driveVel), _done_thresh(doneThresh),
	  _set_state_called_before_output(false) {
	interpolatePoints(waypoints);
	// LOG_F(INFO, "path size: %ld", _path.size());
}

void PurePursuitCommand::setState(const pose_t& pose) {
	this->_pose = pose;
	this->_set_state_called_before_output = true;
}

command_t PurePursuitCommand::getOutput() {
	// LOG_F(INFO, "entering get ouput");
	

    if (!this->_set_state_called_before_output) {
		LOG_F(WARNING, "PurePursuitCommand: getOutput() called before setState() call!");
	}

	double distToGoal = (_pose.head<2>() - _path.back().head<2>()).norm();

	double driveVel = _drive_vel;
	if (distToGoal <= _slow_thresh) {
		driveVel *= distToGoal / _slow_thresh;
	}
	if (distToGoal <= _done_thresh) {
        return {.thetaVel = 0.0, .xVel = 0.0};
    }

	// LOG_F(INFO, "updating index now");
	updateCurrentIndex();
	point_t relIntersect;
	if (_curr_idx >= _path.size() - 1) {
		relIntersect = util::toTransform(_pose).inverse() * _path.back();
	} else {
		relIntersect = lineToCircleIntersection(_path[_curr_idx], _path[_curr_idx + 1]);
	}

	LOG_F(INFO, "Intersect: x=%f y=%f", relIntersect[0], relIntersect[1]);

	double curvature = (2 * relIntersect[1]) 
						/ (relIntersect[0]*relIntersect[0] + relIntersect[1]*relIntersect[1]);

	double thetaVel = _drive_vel * curvature; // curvature times drive vel = theta vel

	LOG_F(INFO, "%f, %f", thetaVel, driveVel);
	return {.thetaVel = 0.5, .xVel = driveVel};
}

point_t PurePursuitCommand::lineToCircleIntersection(point_t& p1, point_t& p2) {
	point_t p1Robot = util::toTransform(_pose).inverse() * p1;
	point_t p2Robot = util::toTransform(_pose).inverse() * p2;
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

	if (disc >= 0) {
		double xSol1 = (D * dy + sgn(dy) * dx * sqrt(disc)) / dr2;
		double xSol2 = (D * dy - sgn(dy) * dx * sqrt(disc)) / dr2;
		double ySol1 = (-D * dx + abs(dy) * sqrt(disc)) / dr2;
		double ySol2 = (-D * dx - abs(dy) * sqrt(disc)) / dr2;

		point_t sol1 = { xSol1, ySol1, 1 };
		point_t sol2 = { xSol2, ySol2, 1 };
	
		double t1 = ((xSol1 - x1)*dx + (ySol1 - y1)*dy) / dr2;
		double t2 = ((xSol2 - x1)*dx + (ySol2 - y1)*dy) / dr2;

		bool valid1 = (t1 >= 0 && t1 <= 1 && xSol1 >= 0);
		bool valid2 = (t2 >= 0 && t2 <= 1 && xSol2 >= 0);

		if (valid1 && !valid2) return sol1;
		if (valid2 && !valid1) return sol2;
		if (valid1 && valid2) return (t1 < t2 ? sol1 : sol2);
		return p2Robot;
	} else {
		return p2Robot;
	}
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
			double t = (numPts * _dist_between_points - accumulatedDist) / segmentLen;
			point_t newPt = {p1[0] + t * (p2[0] - p1[0]), p1[1] + t * (p2[1] - p1[1]), 1};
			_path.push_back(newPt);
			lastPathPoint = newPt;
			numPts++;
		}

		accumulatedDist += segmentLen;
	}
	_path.push_back(waypoints.back());
}

bool PurePursuitCommand::isDone() {
	double distance = (_pose.topRows<2>() - _path.back().topRows<2>()).norm();
	// LOG_F(INFO, "dist to goal %f", distance);
	return distance <= _done_thresh;
}

void PurePursuitCommand::updateCurrentIndex() {
	double distToNext;
	double distToPrev;
	while (_curr_idx < _path.size() - 2) {
		distToNext = (_pose.topRows<2>() - _path[_curr_idx + 1].topRows<2>()).norm();
		if (distToNext < _lookahead_dist) {
			LOG_F(INFO, "updating index");
			_curr_idx++;
		} else {
			break;
		}
	}
}
} // namespace commands
