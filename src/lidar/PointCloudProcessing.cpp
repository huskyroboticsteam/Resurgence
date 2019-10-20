#include "PointCloudProcessing.h"

#include <iostream>
#include <cmath>
#include <set>

constexpr float obstDistThreshold = 0.25;

namespace Lidar
{

float distance(std::pair<float, float> p1, std::pair<float, float> p2)
{
    return sqrtf(powf(p1.first - p2.first, 2) + powf(p1.second - p2.second, 2));
}

// pts should all be as x, y pairs
void clusterPoints(std::vector<std::pair<float, float>> pts,
    std::vector<std::set<std::pair<float, float>>> clusters)
{
    while (!pts.empty())
    {
        std::pair<float, float> curr = pts[0];
        pts.erase(pts.begin());

        int nearestCluster = -1;
        float minDist = std::numeric_limits<float>::infinity();
        for (int i = 0; i < clusters.size(); i++)
        {
            for (std::pair<float, float> xyPoint: clusters[i])
            {
                float dist = distance(curr, xyPoint);
                if (dist < minDist)
                {
                    minDist = dist;
                    nearestCluster = i;
                }
            }
        }
        if (minDist < obstDistThreshold)
        {
            clusters[nearestCluster].insert(curr);
        }
        else
        {
            clusters.push_back(std::set<std::pair<float, float>>());
            clusters.back().insert(curr);
        }
    }
}

} // namespace Lidar
 