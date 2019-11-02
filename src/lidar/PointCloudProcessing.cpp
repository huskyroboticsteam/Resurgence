#include "PointCloudProcessing.h"
#include "SyntheticLidar.h"

#include <cmath>
#include <limits>
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

void getBoundingBoxes(std::set<std::shared_ptr<PointXY>> cluster)
{
    float xmin = std::numeric_limits<float>::infinity();
    float xmax = -std::numeric_limits<float>::infinity();
    float ymin = xmin;
    float ymax = xmax;
    for (std::shared_ptr<PointXY> p: cluster)
    {
        xmin = fmin(xmin, p->x);
        xmax = fmax(xmax, p->x);
        ymin = fmin(ymin, p->y);
        ymax = fmax(ymax, p->y);
    }
    std::cout << "(" << xmin << ", " << ymin << ") -> (" << xmax << ", " << ymax << ")";
    std::cout << std::endl;
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

} // namespace Lidar
