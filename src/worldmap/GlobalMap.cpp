#include "GlobalMap.h"

GlobalMap::GlobalMap() : points()
{
}

points_t GlobalMap::getPoints() const
{
	return points;
}

void GlobalMap::addPoints(const transform_t &trf, const points_t &toAdd)
{
	transform_t trfInv = trf.inverse();
	points_t transformed;
	for (const point_t &point : toAdd)
	{
		if (point(2) != 0)
		{
			transformed.push_back(trfInv * point);
		}
	}

	points_t corrected = correctPointCloud(transformed);

	points.insert(points.end(), corrected.begin(), corrected.end()); // append to global map
}

points_t GlobalMap::correctPointCloud(const points_t &pointCloud)
{
	// TODO: implemented trimmed ICP
	return pointCloud;
}
