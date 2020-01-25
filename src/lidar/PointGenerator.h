#pragma once

#include "PointCloudProcessing.h"

namespace Lidar
{
std::set<std::shared_ptr<PointXY>> generateClusterRadius(float x0, float y0, float r, int num_pts);
std::set<std::shared_ptr<PointXY>> generateClusterLinear(float x0, float y0, float x1, float y1,
                                                         float tol, int num_pts);
} // namespace Lidar
