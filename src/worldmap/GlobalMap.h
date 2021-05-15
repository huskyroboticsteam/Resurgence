#pragma once

#include <vector>

#include "../simulator/utils.h"
#include "QuadTree.h"
#include "TrICP.h"

/**
 * Represents a 2D global point cloud map. As the robot moves around, it will store
 * information about its environment for mapping purposes.
 */
class GlobalMap
{
public:
	/**
	 * Creates a new GlobalMap object, for use with building global lidar point clouds.
	 *
	 * @param areaSize The side length of the square represented by this global map. The units
	 * 			don't matter, as long as they're consistent with the data.
	 * @param maxIter Maximum iterations allowed for fitting new samples to the map
	 * @param relErrChangeThresh Minimum relative error change threshold for fitting new
	 * samples to the map. Iteration will terminate early if the relative change in error
	 * drops below this amount.
	 */
	explicit GlobalMap(double areaSize, int maxIter = 25, double relErrChangeThresh = 0.005);

	/**
	 * Register the given points into the global map.
	 *
	 * @param robotTrf The transform of the robot pose.
	 * @param points The lidar sample point cloud, in the reference frame of the given robot
	 * transform.
	 * @param overlap The estimated overlap, in the range [0,1] between this sample and the
	 * global map. It's better to underestimate than overestimate. However, if it's too low
	 * then no meaningful features can be matched. Set to 0 to add points directly
	 * with no feature matching.
	 */
	void addPoints(const transform_t &robotTrf, const points_t &points, double overlap);

	/**
	 * Get the points in the global map. This vector is a copy, so modifying it will not
	 * harm this class.
	 *
	 * @return A vector of points in the 2D point map.
	 */
	points_t getPoints() const;

	/**
	 * Get the point in the point cloud closest to the given point.
	 *
	 * @param point The point for which the closest point will be found.
	 * @return The closest point, or [0,0,0] if no points are in this map.
	 */
	point_t getClosest(const point_t &point) const;

	/**
	 * Gets all points in the map within the given distance (inclusive) from the given point.
	 *
	 * @param point The point to find the nearby points for.
	 * @param dist The distance within which to search.
	 * @return A vector of points that are within the given distance from the given point.
	 */
	points_t getPointsWithin(const point_t &point, double dist) const;

	/**
	 * Checks if the given point has at least one point within the given distance from it.
	 * Although it's functionally equivalent to checking the size of the vector returned by
	 * getPointsWithin(), it is much cheaper to call this method.
	 *
	 * @param point The point to check if it has a nearby point.
	 * @param dist The distance within which to search.
	 * @return True iff there exists a point in the map, which may be the given point, within
	 * the given distance from the given point. False otherwise.
	 */
	bool hasPointWithin(const point_t &point, double dist) const;

private:
	QuadTree tree;
	TrICP icp;
};
