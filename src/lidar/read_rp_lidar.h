#pragma once

#include <RPLidar/rplidar.h>

#include "../navtypes.h"
#include "../world_interface/data.h"

namespace lidar {

/**
 * @brief Startsup RPLidar with default settings
 */
bool initializeLidar();

/**
 * @brief Provides RP Lidar data at current timeframe
 */
DataPoint<navtypes::points_t> readLidar();

/**
 * @brief Polar struct that holds radians, and theta
 */
struct Polar2D {
	double r, theta;
	double asCartesianX() {
		return r * cos(theta);
	}
	double asCartesianY() {
		return r * sin(theta);
	}
};

}