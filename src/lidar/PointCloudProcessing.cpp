#include "PointCloudProcessing.h"

#include <cmath>
#include <limits>
#include <algorithm>
#include <iostream>

namespace lidar
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
std::vector<std::vector<PointXY>> clusterPoints(
    std::vector<PointXY> &pts, float sep_threshold)
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
std::vector<std::vector<PointXY>> filterPointXYs(std::vector<PointXY> points, float sep_threshold)
{
    std::vector<std::vector<PointXY>> clusters;
    std::vector<PointXY> lastCluster;
    PointXY lastPoint;
    for (int i = 1; i < points.size(); i++)
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
    //checks last point and first point
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

std::vector<PointXY> convexHull(std::vector<PointXY> &cluster)
{
    if (cluster.size() <= 3)
    {
        return cluster;
    }

    // sort points by their x coordinates
    std::sort(std::begin(cluster), std::end(cluster),
              [](PointXY p1, PointXY p2) {
                  return p1.x < p2.x;
              });

    auto orientation = [](PointXY p, PointXY q, PointXY r)
    {
        return (q.x * r.y) - (q.y * r.x) - (p.x * r.y) + (p.y * r.x) + (p.x * q.y) - (p.y * q.x);  
    };

    std::vector<PointXY> hull;
    hull.push_back(cluster[0]);
    hull.push_back(cluster[1]);
    for (int i = 2; i < cluster.size(); i++)
    {
        while (hull.size() >= 2 and orientation(hull[hull.size() - 2],
            hull[hull.size() - 1], cluster[i]) > 0)
        {
            hull.pop_back();
        }
        hull.push_back(cluster[i]);
    }

    for (int i = cluster.size() - 2; i > 0; i--)
    {
        while (hull.size() >= 2 and orientation(hull[hull.size() - 2],
            hull[hull.size() - 1], cluster[i]) > 0)
        {
            hull.pop_back();
        }
        hull.push_back(cluster[i]); 
    }

    return hull;
}

} // namespace Lidar


int main(int argc, char **argv)
{
    using namespace lidar;
    std::vector<PointXY> cluster;
    cluster.push_back({0, 0});
    cluster.push_back({10, 10});
    cluster.push_back({7, 0});
    cluster.push_back({6, 1});
    std::vector<PointXY> hull = convexHull(cluster);
    for (PointXY p: hull)
    {
        std::cout << p.x << " " << p.y << std::endl;
    }
    return 0;
}
