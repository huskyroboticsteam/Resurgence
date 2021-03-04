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
	 * @param maxIter Maximum iterations allowed for fitting new samples to the map
	 * @param relErrChangeThresh Minimum relative error change threshold for fitting new
	 * samples to the map. Iteration will terminate early if the relative change in error
	 * drops below this amount.
	 */
	explicit GlobalMap(int maxIter = 25, double relErrChangeThresh = 0.005);
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
	 * Get the point in the point cloud closest (as determined by L1 norm) to the given point.
	 *
	 * @param point The point for which the closest point will be found.
	 * @return The closest point, or [0,0,0] if no points are in this map.
	 */
	point_t getClosest(const point_t &point) const;

private:
	QuadTree tree;
	transform_t adjustmentTransform;
	transform_t adjustmentTransformInv;
	TrICP icp;
};
