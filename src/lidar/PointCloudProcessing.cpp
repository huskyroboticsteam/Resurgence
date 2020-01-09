#include "PointCloudProcessing.h"

#include <cmath>
#include <limits>
#include <algorithm>
#include <iostream>

namespace Lidar
{

bool approxEqual(PointXY p, PointXY q)
{
    return fabs(p.x - q.x) < 1e-4 && fabs(p.y - q.y) < 1e-4;
}

float distance(float x0, float y0, float x1, float y1)
{
    return sqrtf(powf(x0 - x1, 2) + powf(y0 - y1, 2));
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
std::vector<std::set<PointXY>> clusterPoints(
    std::vector<PointXY> &pts, float sep_threshold)
{
    std::vector<std::set<PointXY>> clusters;
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
            clusters[nearest_cluster].insert(curr);
        }
        else
        {
            clusters.push_back(std::set<PointXY>());
            clusters.back().insert(curr);
        }
    }
    return clusters;
}

// another clustering algorithm
// O(n) time
// assumes points are in order by their polar degree
std::vector<std::set<PointXY>> filterPointXYs(std::vector<PointXY> points, float sep_threshold)
{
    std::vector<std::set<PointXY>> clusters;
    std::set<PointXY> lastCluster;
    PointXY lastPoint;
    for (int i = 1; i < points.size(); i++)
    {
        float diff = distance(points[i].x, points[i].y, points[i - 1].x, points[i - 1].y);
        if (diff < sep_threshold)
        {
            if (approxEqual(lastPoint, points[i - 1]))
            {
                lastCluster.insert(points[i]);
            }
            else
            {
                std::set<PointXY> cluster;
                cluster.insert(points[i - 1]);
                cluster.insert(points[i]);
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
            lastCluster.insert(points[i]);
        }
    }
    //checks last point and first point
    float diff = distance(points[points.size() - 1].x, points[points.size() - 1].y,
                          points[0].x, points[0].y);
    if (diff < sep_threshold)
    {
        if (approxEqual(lastPoint, points[points.size() - 1]))
        {
            lastCluster.insert(points[0]);
        }
        else
        {
            std::set<PointXY> cluster;
            cluster.insert(points[points.size() - 1]);
            cluster.insert(points[0]);
            clusters.push_back(cluster);
        }
    }
    return clusters;
}

void filterGroundPoints(std::vector<Polar2D> &pts, float scan_height,
                        float slope_tol_rad)
{
    std::vector<Polar2D>::iterator itr = pts.begin();
    while (itr != pts.end())
    {
        float slope_rad = tanhf(scan_height / (*itr).r);
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

float orientation(PointXY p, PointXY q, PointXY r)
{
    return (q.x * r.y - q.y * r.x) - (p.x * r.y - p.y * r.x) + (p.x * q.y - p.y * q.x);
}

std::vector<PointXY> convexHull(std::set<PointXY> &cluster)
{
    if (cluster.size() < 3)
    {
        return std::vector<PointXY>();
    }

    // sort points by their x coordinates
    std::vector<PointXY> points(cluster.begin(), cluster.end());
    std::sort(std::begin(points), std::end(points),
              [](std::shared_ptr<PointXY> p1, std::shared_ptr<PointXY> p2) {
                  return p1->x < p2->x;
              });

    std::vector<PointXY> hull;
    hull.push_back(points[0]);
    hull.push_back(points[1]);
    for (int i = 3; i < points.size(); i++)
    {
        if (orientation(*(hull.end() - 1), *hull.end(), points[i]) < 0)
        {
            hull.push_back(points[i]);
        }
        else
        {
            std::vector<PointXY>::iterator itr = hull.end();
            while (orientation(*(hull.end() - 1), *hull.end(), points[i]) >= 0)
            {
                itr = hull.erase(itr);
            }
        }
    }

    return hull;
}

} // namespace Lidar
