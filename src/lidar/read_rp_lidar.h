#pragma once

#include <RPLidar/rplidar.h>

#include "../navtypes.h"
#include "../world_interface/data.h"

namespace lidar {

/**
 * @brief Startsup RPLidar with default settings
 */
bool initializeLidar(double max_dist=16);

/**
 * @brief Provides RP Lidar data at current timeframe
 */
robot::types::DataPoint<navtypes::points_t> readLidar();

/**
 * @brief Polar struct that holds radius, and theta (radians)
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