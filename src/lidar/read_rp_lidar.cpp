#include "read_rp_lidar.h"

#include "../navtypes.h"
#include "../world_interface/world_interface.h"
#include "PointCloudProcessing.h"
#include "PointGenerator.h"

#include <atomic>
#include <mutex>
#include <thread>

std::atomic<bool> lidar_initialized(false);
std::atomic<datatime_t> lidar_time(datatime_t{});
std::thread lidar_thread;
points_t last_points = {};
std::mutex points_lock;

namespace lidar {

bool initializeLidar() {

}

bool isInitialized() {

}

void readLidarLoop() {

}

DataPoint<points_t> readLidar() {

}
}

