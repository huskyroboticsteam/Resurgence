#pragma once

#include <vector>
#include <set>
#include <memory>

namespace Lidar
{

typedef struct PointXY
{
    float x;
    float y;
} PointXY;

std::vector<std::set<std::shared_ptr<PointXY>>> clusterPoints(
    std::vector<std::shared_ptr<PointXY>> pts, float sepThreshold);
}; // namespace Lidar
