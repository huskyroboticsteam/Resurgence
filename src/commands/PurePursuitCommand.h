#pragma once

#include "../navtypes.h"
#include "CommandBase.h"

namespace commands {
class PurePursuitCommand : CommandBase {
public:
	//
	PurePursuitCommand(const navtypes::points_t& waypoints, double driveVel,
					   double doneThresh);

    void setState(const navtypes::pose_t& pose);

	/**
	 *  @brief Finds next relevant target and returns raw heading velocity and forward
	 * 		   velocity. There are no guarantees that the outputs will be within the 
	 * 		   rover's physical limits.
	 * 	@return command_t
	 */
	command_t getOutput();

	/**
	 * 
	 */
    bool isDone();

private:
	/** @internal
	 * 	@brief Finds optimal intersection between _pose and line segment between p1 and p2.
	 * 	   	   Assumes p2 follows p1 in path implementation.
	 * 	@param p1 First point to form line segment
	 * 	@param p2 Second point to form line segment
	 * 	@return "Optimal" intersection as a point. Intersections closer to p2 are prioritized.
	 * 		    If no intersection is found, p2  will be returned as the default.
	 * 
	 * 	@see Formula reference: https://mathworld.wolfram.com/Circle-LineIntersection.html
	 */
	navtypes::point_t lineToCircleIntersection(navtypes::point_t& p1, navtypes::point_t& p2);

	/**
	 * 
	 */
	void interpolatePoints(const navtypes::points_t& waypoints);

	int _curr_idx; // current point target in path
	navtypes::points_t _path; // list of equidistant points representing path

	navtypes::pose_t _pose; // current pose, updated by caller via setState()

	// set Constants --- ? could move to Constants file
	double _drive_vel;
	double _done_thresh;
	double _dist_between_points;
	double _lookahead_dist;

	bool _set_state_called_before_output;
};
} // namespace commands