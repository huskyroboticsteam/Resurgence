#pragma once

#include <vector>

#include <Eigen/Core>

namespace navtypes {

/** Represents a GPS coordinate in degrees. Positive is north/east. */
struct gpscoords_t {
	/** the latitude of the gps coordinate, in degrees */
	double lat;
	/** the longitude of the gps coordinate, in degrees */
	double lon;
};

/* Obstacles are specifed as a list of 2D vertices. Must be convex.
 * Do not repeat the starting vertex at the end. */
using obstacle_t = Eigen::ArrayX2d;
using obstacles_t = std::vector<obstacle_t>;

using pose_t = Eigen::Vector3d;		 // Robot pose: x, y, theta
using transform_t = Eigen::Matrix3d; // a pose in matrix form; see toPose below
using trajectory_t = std::vector<transform_t>;

using point_t =
	Eigen::Vector3d; // x, y, 1 (or 0, 0, 0 to represent "no data")
					 // The 1 at the end makes calculating affine transforms easier.
using points_t = std::vector<point_t>;
using traj_points_t = std::vector<points_t>; // points_t for each time along a trajectory

struct URCLeg {
	int left_post_id;
	int right_post_id; // This will be -1 for legs that are just single posts
	point_t approx_GPS;
};

struct URCLegGPS {
	int left_post_id;
	int right_post_id;
	gpscoords_t gps;
};

} // namespace navtypes