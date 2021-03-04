#include "GlobalMap.h"

#include <Eigen/LU>

/*
 * Since the map is transformed to fit the sample, and there might be a lot of points in the
 * global map, we don't transform the entire map with each sample. Instead, we only transform
 * the map when it is requested by the client.
 *
 * We have a notion of "naive" map space and "corrected" map space. Both are in the map
 * reference frame, but "naive" map space is when every sample is transformed to fit the first,
 * while the "corrected" map space is when every sample is transformed to fit the last.
 * The "corrected" map space is so named because it most correctly matches the robot's
 * pose estimation.
 *
 * Points are stored in "naive" map space, and the adjustmentTransform transformation maps
 * from "naive" to "corrected" map space.
 */

GlobalMap::GlobalMap(int maxIter, double relErrChangeThresh)
	: adjustmentTransform(transform_t::Identity()),
	  adjustmentTransformInv(transform_t::Identity()),
	  tree(10000),
	  icp(maxIter, relErrChangeThresh,
		  std::bind(&GlobalMap::getClosest, this, std::placeholders::_1))
{
}

points_t GlobalMap::getPoints() const
{
	points_t corrected = tree.getAllPoints();
	for (point_t &p : corrected)
	{
		p = adjustmentTransform * p;
	}
	return corrected;
}

void GlobalMap::addPoints(const transform_t &robotTrf, const points_t &toAdd, double overlap)
{
	if (toAdd.empty())
	{
		return;
	}
	transform_t trfInv = robotTrf.inverse();
	points_t transformed;
	for (const point_t &point : toAdd)
	{
		if (point(2) != 0)
		{
			transformed.push_back(trfInv * point);
		}
	}

	if (!tree.empty())
	{
		if (overlap > 0)
		{
			transform_t mapAdjustment = icp.correct(transformed, overlap);
			// instead of adjusting the entire map right now, store the transformation
			adjustmentTransform = mapAdjustment * adjustmentTransform;
			adjustmentTransformInv = adjustmentTransform.inverse();
		}
		for (point_t &p : transformed)
		{
			p = adjustmentTransformInv * p;
		}
	}

	// add to global map
	for (const point_t &p : transformed) {
		tree.add(p);
	}
}

point_t GlobalMap::getClosest(const point_t &point) const
{
	// convert the lookup point to "naive" map space
	point_t p = adjustmentTransformInv * point;
	if (tree.empty())
	{
		return {0, 0, 0};
	}
	point_t closest = tree.getClosest(p);
	// convert back to "corrected" map space
	return adjustmentTransform * closest;
}
