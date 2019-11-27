#include "PointCloudProcessing.h"
#include "SyntheticLidar.h"

#include <cmath>
#include <limits>
#include <algorithm>
#include <iostream>

namespace Lidar
{

float distance(float x0, float y0, float x1, float y1)
{
    return sqrtf(powf(x0 - x1, 2) + powf(y0 - y1, 2));
}

// p is a point relative to the robot
// heading should be in radians, 0 is north, pi/2 is east, pi is south, 3*pi/2 is west
void localToGlobal(std::shared_ptr<PointXY> p, float x_loc, float y_loc, float heading)
{
    p->x = x_loc - (cosf(heading) * p->x - sinf(heading) * p->y);
    p->y = y_loc - (sinf(heading) * p->x + cosf(heading) * p->y);
}

std::vector<std::set<std::shared_ptr<PointXY>>> clusterPoints(
    std::vector<std::shared_ptr<PointXY>> pts, float sep_threshold)
{
    std::vector<std::set<std::shared_ptr<PointXY>>> clusters;
    for (std::shared_ptr<PointXY> curr : pts)
    {
        int nearest_cluster = -1;
        float min_dist = std::numeric_limits<float>::infinity();
        for (int i = 0; i < clusters.size(); i++)
        {
            for (std::shared_ptr<PointXY> xyPoint : clusters[i])
            {
                float dist = distance(curr->x, curr->y, xyPoint->x, xyPoint->y);
                if (dist < min_dist)
                {
                    min_dist = dist;
                    nearest_cluster = i;
                }
            }
        }
        if (min_dist < sep_threshold)
        {
            clusters[nearest_cluster].insert(curr);
        }
        else
        {
            clusters.push_back(std::set<std::shared_ptr<PointXY>>());
            clusters.back().insert(curr);
        }
    }
    return clusters;
}

void filterGroundPoints(std::vector<std::shared_ptr<Polar2D>> pts, float scan_height,
                        float slope_tol_rad)
{
    std::vector<std::shared_ptr<Polar2D>>::iterator itr = pts.begin();
    while (itr != pts.end())
    {
        float slope_rad = tanhf(scan_height / (*itr)->r);
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

float orientation(std::shared_ptr<PointXY> p, std::shared_ptr<PointXY> q,
                  std::shared_ptr<PointXY> r)
{
    return (q->x * r->y - q->y * r->x) - (p->x * r->y - p->y * r->x) + (p->x * q->y - p->y * q->x);
}

std::vector<std::shared_ptr<PointXY>> convexHull(std::set<std::shared_ptr<PointXY>> cluster)
{
    if (cluster.size() < 3)
    {
        return std::vector<std::shared_ptr<PointXY>>();
    }

    // sort points by their x coordinates
    std::vector<std::shared_ptr<PointXY>> points(cluster.begin(), cluster.end());
    std::sort(std::begin(points), std::end(points), [](std::shared_ptr<PointXY> p1, std::shared_ptr<PointXY> p2) { return p1->x > p2->x; });

    std::vector<std::shared_ptr<PointXY>> hull;
    hull.push_back(points[0]);
    hull.push_back(points[1]);
    for (int i = 3; i < points.size(); i++)
    {
        while (orientation(points[i], *(hull.end()), *(hull.end() - 1)) < 0)
        {
            hull.erase(hull.end());
        }
        hull.push_back(points[i]);
    }

    return hull;
}

} // namespace Lidar
