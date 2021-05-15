#include "GlobalMap.h"

#include <Eigen/LU>

/*
 * The map is stored relative to the first sample. This isn't ideal, but
 * transforming the map to fit the latest sample worked terribly.
 */

GlobalMap::GlobalMap(double areaSize, int maxIter, double relErrChangeThresh)
	: tree(areaSize),
	  icp(maxIter, relErrChangeThresh,
		  std::bind(&GlobalMap::getClosest, this, std::placeholders::_1))
{
}

points_t GlobalMap::getPoints() const
{
	return tree.getAllPoints();
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
			// just transform the sample instead of the map (yes i know there are some
			// unnecessary inversions going on but I'll fix that later)
			// TODO: fix the double inversions
			transform_t adj = mapAdjustment.inverse();
			for (point_t &p : transformed)
			{
				p = adj * p;
			}
		}
	}

	// add to global map
	for (const point_t &p : transformed) {
		assert(tree.add(p));
	}
}

point_t GlobalMap::getClosest(const point_t &point) const
{
	if (tree.empty())
	{
		return {0, 0, 0};
	}
	point_t closest = tree.getClosest(point);
	return closest;
}

points_t GlobalMap::getPointsWithin(const point_t &point, double dist) const
{
	points_t points;
	if (!tree.empty())
	{
		points_t within = tree.getPointsWithin(point, dist * 2);
		for (const point_t &p1 : within) {
			if ((p1 - point).topRows<2>().norm() <= dist) {
				points.push_back(p1);
			}
		}
	}
	return points;
}

bool GlobalMap::hasPointWithin(const point_t &point, double dist) const
{
	if (tree.empty()) {
		return false;
	}
	points_t within = tree.getPointsWithin(point, dist * 2);
	for (const point_t &p : within) {
		if ((p - point).topRows<2>().norm() <= dist) return true;
	}
	return false;
}
