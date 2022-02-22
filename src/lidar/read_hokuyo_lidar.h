#pragma once

#include "../navtypes.h"
#include "../world_interface/data.h"

namespace lidar {
/**
 * @brief Initialize the lidar sensor, must be called before any other lidar operations.
 *
 * @returns true iff the lidar was initialized successfully.
 */
bool initializeLidar();

/**
 * @brief Check if the lidar was initialized.
 *
 * @returns true iff the lidar was initialized successfully.
 */
bool isLidarInitialized();

/**
 * @brief Get the latest lidar data.
 * May return the same data twice if called again too quickly, before new data was collected.
 *
 * @return The data from the lidar sensor, or an empty data point if the lidar wasn't initialized.
 */
robot::types::DataPoint<navtypes::points_t> readLidar();
} // namespace lidar
