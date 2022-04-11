#pragma once

#include <RPLidar/rplidar.h>

#include "../navtypes.h"
#include "../world_interface/data.h"

namespace lidar {

bool initializeLidar();

void readLidarLoop();

typedef struct Polar2D {
	double r, theta;
	double asCartesianX() {
		return r * cos(theta);
	}
	double asCartesianY() {
		return r * sin(theta);
	}
} Polar2D;

}