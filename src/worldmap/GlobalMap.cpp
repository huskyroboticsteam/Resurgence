#include "GlobalMap.h"

#include <Eigen/LU>

GlobalMap::GlobalMap(double areaSize, int scanStride, int maxIter, double relErrChangeThresh)
	: tree(areaSize), icp(maxIter, relErrChangeThresh,
						  std::bind(&GlobalMap::getClosest, this, std::placeholders::_1)),
	  scanStride(scanStride)
{
}

points_t GlobalMap::getPoints() const
{
	return tree.getAllPoints();
}

size_t GlobalMap::size() const
{
	return tree.getSize();
}

void GlobalMap::addPoints(const transform_t &robotTrf, const points_t &toAdd, double overlap)
{
	if (toAdd.empty())
	{
		return;
	}
	transform_t trfInv = robotTrf.inverse();
	points_t transformed;
	for (size_t i = 0; i < toAdd.size(); i += scanStride)
	{
		const point_t &point = toAdd[i];
		if (point(2) != 0)
		{
			transformed.push_back(trfInv * point);
		}
	}

	if (!tree.empty())
	{
		if (overlap > 0)
		{
			// find the transformation from the sample to the map
			transform_t adj = icp.correct(transformed, overlap);
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
	return tree.hasPointWithin(point, dist);
}
