#pragma once
#include <rplidar.h>

#include "../navtypes.h"
#include "../world_interface/data.h"

namespace lidar {

bool initializeLidar();

bool isInitialized();

void readLidarLoop();

DataPoint<points_t> readLidar();
}
}