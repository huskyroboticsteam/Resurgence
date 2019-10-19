#pragma once

#include <vector>
#include <set>

namespace Lidar
{
void clusterPoints(std::vector<std::pair<float, float>> pts,
                   std::vector<std::set<std::pair<float, float>>> clusters);
}; // namespace Lidar
