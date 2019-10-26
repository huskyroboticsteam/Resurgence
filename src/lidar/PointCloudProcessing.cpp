#include "PointCloudProcessing.h"

#include <cmath>
#include <limits>

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

void getBoundingBoxes(std::vector<std::set<std::shared_ptr<PointXY>>> clusters)
{
    for (std::set<std::shared_ptr<PointXY>> cluster: clusters)
    {
        
    }
}

} // namespace Lidar
