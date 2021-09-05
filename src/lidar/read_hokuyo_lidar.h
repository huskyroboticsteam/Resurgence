#pragma once

#include "../simulator/utils.h"

namespace lidar {
bool initializeLidar();
bool isLidarInitialized();
points_t readLidar();
bool isLidarDataFresh();
} // namespace lidar
