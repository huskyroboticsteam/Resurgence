#pragma once

#include "lidar/SyntheticLidar.h"

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

float distance(float x0, float y0, float x1, float y1);

std::vector<std::set<std::shared_ptr<PointXY>>> clusterPoints(
    std::vector<std::shared_ptr<PointXY>> pts, float sepThreshold);

void filterGroundPoints(std::vector<std::shared_ptr<Polar2D>> pts, float scan_height,
                        float slope_tol_rad);

std::vector<BoundingBox> boundCluster(std::set<std::shared_ptr<PointXY>> cluster);
}; // namespace Lidar
