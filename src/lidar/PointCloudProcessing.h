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

typedef struct BoundingBox
{
    float xmin;
    float ymin;
    float xmax;
    float ymax;
} BoundingBox;

std::vector<std::set<std::shared_ptr<PointXY>>> clusterPoints(
    std::vector<std::shared_ptr<PointXY>> pts, float sepThreshold);

std::vector<BoundingBox> boundCluster(std::set<std::shared_ptr<PointXY>> cluster);
}; // namespace Lidar
