#include "GlobalMap.h"
#include "TrICP.h"

GlobalMap::GlobalMap() : points()
{
}

const points_t &GlobalMap::getPoints() const
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

// https://www.researchgate.net/publication/3974183_The_Trimmed_Iterative_Closest_Point_algorithm
points_t GlobalMap::correctPointCloud(const points_t &pointCloud)
{
	TrICP icp(*this);
	return icp.correct(pointCloud);
}

point_t GlobalMap::getClosest(const point_t &point) const
{
	if (points.empty())
	{
		return {0, 0, 0};
	}
	// TODO: optimize with geohashing
	point_t closest = points[0];
	double minDist = (point - closest).norm();
	for (int i = 1; i < points.size(); i++) {
		double dist = (point - points[i]).norm();
		if (dist < minDist) {
			minDist = dist;
			closest = points[i];
		}
	}
	return closest;
}
