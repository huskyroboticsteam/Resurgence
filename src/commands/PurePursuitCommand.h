#pragma once

#include "../navtypes.h"
#include "CommandBase.h"

#include <fstream>

namespace commands {
class PurePursuitCommand : CommandBase {
public:
	//
	PurePursuitCommand(const navtypes::points_t waypoints);

	~PurePursuitCommand();

	/**
	 * @brief Updates pose.
	 * @param pose new pose
	 */
    void setState(const navtypes::pose_t& pose);

	/**
	 * @brief Finds next relevant target and returns raw heading velocity and forward
	 * 		   velocity. There are no guarantees that the outputs will be within the
	 * 		   rover's physical limits.
	 * @return command_t containing forward and angular velocities calculated by Pure Pursuit
	 * 		   in the form { .thetaVel, .xVel }
	 */
	command_t getOutput();

	/**
	 * @brief Returns whether or not the last waypoint has been reached AND whether the waypoint
	 * 		  has been reached for at least five iterations of the control loop.
	 * 		  Call only once per call loop; otherwise, there will be unintended effects.
	 */
    bool isDone();

	void reset();

private:
	/** @internal
	 * @brief Finds optimal intersection between _pose and line segment between p1 and p2.
	 *   	  Assumes p2 follows p1 in path.
	 * @param p1 First point to form line segment
	 * @param p2 Second point to form line segment
	 * @return "Optimal" intersection as a point. Intersections closer to p2 are prioritized.
	 * 		   If no intersection is found, p2  will be returned as the default.
	 *
	 * @see Formula reference: https://mathworld.wolfram.com/Circle-LineIntersection.html
	 */
	navtypes::point_t lineToCircleIntersection(const navtypes::point_t& p1,
											   const navtypes::point_t& p2);

	/** @internal
	 * @brief Takes given waypoints and interpolates between them to form equidistant points.
	 *		  Distance between points is Constants::commands::DIST_BETWEEN_POINTS
	 *		  Sets result to _path member.
	 * @param waypoints vector of waypoints to interpolate
	 */
	void interpolatePoints(const navtypes::points_t& waypoints);

	/** @internal
	 * @brief Checks distance from next index's point and updates _curr_idx if distance is less
	 * 		  than the lookahead (Constants::commands::LOOKAHEAD_DIST). Will update idx as many 
	 * 		  times as necessary until the condition: distance to next idx point >= lookahead 
	 * 		  distance is met.
	 */
	void updateCurrentIndex();

	int _curr_idx = 0; 		  // current point target in path
	navtypes::points_t _path; // list of equidistant points representing path

	navtypes::pose_t _pose;   // current pose, updated by caller via setState()

	// set Constants --- ? could move to Constants file
	double _drive_vel = 3.0;
	double _slow_thresh = 3.0;

	double _done_thresh = 1.5;
	int _done_count = 0;

	bool _set_state_called_before_output = false;

	std::ofstream file;
};
} // namespace commands