#include "GlobalMap.h"

GlobalMap::GlobalMap()
	: points(), adjustmentTransform(transform_t::Identity()),
	  adjustmentTransformInv(transform_t::Identity()),
	  icp(25, 0.005, std::bind(&GlobalMap::getClosest, this, std::placeholders::_1))
{
}

points_t GlobalMap::getPoints() const
{
	points_t corrected = points;
	for (point_t &p : corrected)
	{
		p = adjustmentTransform * p;
	}
	return corrected;
}

void GlobalMap::addPoints(const transform_t &robotTrf, const points_t &toAdd, double overlap)
{
	transform_t trfInv = robotTrf.inverse();
	points_t transformed;
	for (const point_t &point : toAdd)
	{
		if (point(2) != 0)
		{
			transformed.push_back(trfInv * point);
		}
	}

	if (!points.empty())
	{
		transform_t mapAdjustment = icp.correct(transformed, overlap);
		// instead of adjusting the entire map right now, store the transformation
		adjustmentTransform = mapAdjustment * adjustmentTransform;
		adjustmentTransformInv = adjustmentTransform.inverse();
		for (point_t &p : transformed)
		{
			p = adjustmentTransformInv * p;
		}
	}

	// append to global map
	points.insert(points.end(), transformed.begin(), transformed.end());
}

// points_t GlobalMap::correctPointCloud(const points_t &pointCloud)
//{
//	TrICP icp(10, 0.01, std::bind(&GlobalMap::getClosest, this, std::placeholders::_1));
//	return icp.correct(pointCloud, 0.4);
//}

point_t GlobalMap::getClosest(const point_t &point) const
{
	// convert the lookup point to "naive" map space
	point_t p = adjustmentTransformInv * point;
	if (points.empty())
	{
		return {0, 0, 0};
	}
	// TODO: optimize with geohashing
	point_t closest = points[0];
	double minDist = (p - closest).norm();
	for (int i = 1; i < points.size(); i++)
	{
		double dist = (p - points[i]).norm();
		if (dist < minDist)
		{
			minDist = dist;
			closest = points[i];
		}
	}
	// convert back to "corrected" map space
	return adjustmentTransform * closest;
}
