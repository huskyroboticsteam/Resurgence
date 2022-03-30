#pragma once

#include <RPLidar/rplidar.h>

#include "../navtypes.h"
#include "../world_interface/data.h"

namespace lidar {

bool initializeLidar();

void readLidarLoop();

DataPoint<points_t> readLidar();
}