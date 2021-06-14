#include "PointCloudProcessing.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include "URGLidar.h"

namespace lidar
{

const double LIDAR_METERS_PER_TICK = 0.001; // 1 mm resolution

bool approxEqual(PointXY p, PointXY q)
{
	return fabs(p.x - q.x) < 1e-4 && fabs(p.y - q.y) < 1e-4;
}

float distance(float x0, float y0, float x1, float y1)
{
	return sqrtf(powf(x0 - x1, 2) + powf(y0 - y1, 2));
}

PointXY polarToCartesian(Polar2D p)
{
	return PointXY(
		{static_cast<float>(p.r * cos(p.theta)), static_cast<float>(p.r * sin(p.theta))});
}

point_t polarToCartesian2(Polar2D p)
{
	double x = p.r * cos(p.theta - M_PI/2);
	double y = p.r * sin(p.theta - M_PI/2);
	point_t ret({x*LIDAR_METERS_PER_TICK, y*LIDAR_METERS_PER_TICK, 1.0});
	return ret;
}

// p is a point relative to the robot
// heading should be in radians, 0 is north, pi/2 is east, pi is south, 3*pi/2 is west
void localToGlobal(PointXY &p, float x_loc, float y_loc, float heading)
{
	p.x = x_loc - (cosf(heading) * p.x - sinf(heading) * p.y);
	p.y = y_loc - (sinf(heading) * p.x + cosf(heading) * p.y);
}

// clustering algorithm
// O(n^2) time
// does not assume any ordering or patterns among points
std::vector<std::vector<PointXY>> clusterPoints(std::vector<PointXY> &pts, float sep_threshold)
{
	std::vector<std::vector<PointXY>> clusters;
	for (PointXY curr : pts)
	{
		int nearest_cluster = -1;
		float min_dist = std::numeric_limits<float>::infinity();
		for (int i = 0; i < clusters.size(); i++)
		{
			for (PointXY xyPoint : clusters[i])
			{
				float dist = distance(curr.x, curr.y, xyPoint.x, xyPoint.y);
				if (dist < min_dist)
				{
					min_dist = dist;
					nearest_cluster = i;
				}
			}
		}
		if (min_dist < sep_threshold)
		{
			clusters[nearest_cluster].push_back(curr);
		}
		else
		{
			clusters.push_back(std::vector<PointXY>());
			clusters[clusters.size() - 1].push_back(curr);
		}
	}
	return clusters;
}

// another clustering algorithm
// O(n) time
// assumes points are in order by their polar degree
std::vector<std::vector<PointXY>> clusterOrderedPoints(std::vector<PointXY> &points,
													   float sep_threshold)
{
	std::vector<std::vector<PointXY>> clusters;
	std::vector<PointXY> lastCluster;
	for (int i = 1; i < points.size(); i++)
	PointXY lastPoint = points[0];
	{
		float diff = distance(points[i].x, points[i].y, points[i - 1].x, points[i - 1].y);
		if (diff < sep_threshold)
		{
			if (approxEqual(lastPoint, points[i - 1]))
			{
				lastCluster.push_back(points[i]);
			}
			else
			{
				std::vector<PointXY> cluster;
				cluster.push_back(points[i - 1]);
				cluster.push_back(points[i]);
				lastPoint = points[i];
				clusters.push_back(cluster);
				lastCluster = cluster;
			}
		}
		PointXY zero_pt;
		zero_pt.x = 0;
		zero_pt.y = 0;
		if (!approxEqual(lastPoint, zero_pt) &&
			distance(points[i].x, points[i].y, lastPoint.x, lastPoint.y) < sep_threshold)
		{
			lastCluster.push_back(points[i]);
		}
	}
	// checks last point and first point
	float diff = distance(points[points.size() - 1].x, points[points.size() - 1].y,
						  points[0].x, points[0].y);
	if (diff < sep_threshold)
	{
		if (approxEqual(lastPoint, points[points.size() - 1]))
		{
			lastCluster.push_back(points[0]);
		}
		else
		{
			std::vector<PointXY> cluster;
			cluster.push_back(points[points.size() - 1]);
			cluster.push_back(points[0]);
			clusters.push_back(cluster);
		}
	}
	return clusters;
}

// removes points from the pts list that are ground points
// pitch_rad is positive if learning forward, else negative
// scan_height must be in the same units as the points distance
void filterGroundPoints(std::vector<Polar2D> &pts, float scan_height, float pitch_rad,
						float slope_tol_rad)
{
	std::vector<Polar2D>::iterator itr = pts.begin();
	while (itr != pts.end())
	{
		float slope_rad = M_PI - pitch_rad - atan((*itr).r / scan_height);
		if (slope_rad < slope_tol_rad)
		{
			itr = pts.erase(itr);
		}
		else
		{
			itr++;
		}
	}
}

std::vector<PointXY> convexHull(std::vector<PointXY> &cluster)
{
	if (cluster.size() <= 3)
	{
		return cluster;
	}

	// sort points by their x coordinates
	std::sort(std::begin(cluster), std::end(cluster),
			  [](PointXY p1, PointXY p2) { return p1.x < p2.x; });

	// 0 -> collinear, positive -> counterclockwise, negative -> clockwise
	auto orientation = [](PointXY p, PointXY q, PointXY r) {
		return (q.x * r.y) - (q.y * r.x) - (p.x * r.y) + (p.y * r.x) + (p.x * q.y) -
			   (p.y * q.x);
	};

	std::vector<PointXY> top_hull;
	std::vector<PointXY> bot_hull;
	for (int i = 0; i < cluster.size(); i++)
	{
		while (top_hull.size() >= 2 &&
			   orientation(top_hull[top_hull.size() - 2], top_hull[top_hull.size() - 1],
						   cluster[i]) > 0)
		{
			top_hull.pop_back();
		}
		top_hull.push_back(cluster[i]);
	}

	for (int i = cluster.size() - 1; i >= 0; i--)
	{
		while (bot_hull.size() >= 2 &&
			   orientation(bot_hull[bot_hull.size() - 2], bot_hull[bot_hull.size() - 1],
						   cluster[i]) > 0)
		{
			bot_hull.pop_back();
		}
		bot_hull.push_back(cluster[i]);
	}

	top_hull.insert(top_hull.end(), bot_hull.begin() + 1, bot_hull.end() - 1);
	return top_hull;
}

} // namespace lidar
