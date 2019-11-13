#include "PointCloudProcessing.h"
#include "SyntheticLidar.h"

#include <cmath>
#include <limits>
#include <queue>
#include <iostream>

namespace Lidar
{

float distance(std::shared_ptr<PointXY> p1, std::shared_ptr<PointXY> p2)
{
    return sqrtf(powf(p1->x - p2->x, 2) + powf(p1->y - p2->y, 2));
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
                float dist = distance(curr, xyPoint);
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

std::vector<BoundingBox> boundCluster(std::set<std::shared_ptr<PointXY>> cluster)
{
    std::vector<BoundingBox> bounds;
    auto xcmp = [](std::shared_ptr<PointXY> p1, std::shared_ptr<PointXY> p2) {
        return p1->x > p2->x;
    };
    auto ycmp = [](std::shared_ptr<PointXY> p1, std::shared_ptr<PointXY> p2) {
        return p1->y > p2->y;
    };
    std::priority_queue<std::shared_ptr<PointXY>, std::vector<std::shared_ptr<PointXY>>,
        decltype(xcmp)> x_pq(xcmp);
    std::priority_queue<std::shared_ptr<PointXY>, std::vector<std::shared_ptr<PointXY>>,
        decltype(ycmp)> y_pq(ycmp);
    for (std::shared_ptr<PointXY> p: cluster)
    {
        x_pq.push(p);
        y_pq.push(p);
    }



    return bounds;
}
} // namespace Lidar
