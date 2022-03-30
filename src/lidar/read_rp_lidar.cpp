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
RPLidar rp_lidar;
std::thread lidar_thread;
points_t last_points = {};
std::mutex points_lock;

namespace lidar {

bool initializeLidar() {
    if (!lidar_initialized) {
        rp_lidar("/dev/ttyUSB0");
        if (!rp_lidar.checkHealth()) {
            perror("failed to connect to rp lidar");
        }

        rp_lidar.setBaudrate(115200);
        rp_lidar.setMaxDistance(16.0);
        lidar_thread = std::thread(&readLidarLoop);
    }
}

void readLidarLoop() {
    using namespace std::chrono;
    while (true) {
        if (auto scan = rp.poll()) {
            std::cout << *scan << std::endl;
        } else {
            std::this_thread::sleep_for(milliseconds(10)));
        }
    } 
}

DataPoint<points_t> readLidar() {
    if (lidar_initialized) {
        
    }
}
} // namespace lidar

