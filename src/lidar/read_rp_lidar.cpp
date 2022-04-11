#include "read_rp_lidar.h"

#include "../navtypes.h"
#include "../world_interface/world_interface.h"
#include "PointCloudProcessing.h"
#include "PointGenerator.h"

#include <atomic>
#include <mutex>
#include <thread>
#include <iostream>

using namespace navtypes;

std::atomic<bool> lidar_initialized(false);
RPLidar rp_lidar("/dev/ttyUSB0");
std::thread lidar_thread;
points_t last_points = {};
std::mutex points_lock;

namespace lidar {

bool initializeLidar() {
    if (!lidar_initialized) {
        if (!rp_lidar.checkHealth()) {
            perror("failed to connect to rp lidar");
        }

        rp_lidar.setBaudrate(115200);
        rp_lidar.setMaxDistance(16.0);
        lidar_thread = std::thread(&readLidarLoop);
    }
    return lidar_initialized;
}

void readLidarLoop() {
    using namespace std::chrono;
    std::vector<Polar2D> polarPts;    
    while (true) {
        auto scan = rp_lidar.poll();
        if (!scan) {
            perror("failed to get frame");
            continue;
        }

        // Converts data into Polar Coord
        std::vector<Polar2D> currFrames;
        for (int i = 0; i < scan.value().ranges.size(); i++) {
            double dtheta = (scan.value().angle_max-scan.value().angle_min)/(scan.value().ranges.size()-1);
            double rad = dtheta*i;
            double dist = scan.value().ranges[i];

            Polar2D frame{dist, dtheta};
            currFrames.push_back(frame);
        }
    
        // Converts to regular points
        std::vector<point_t> pts(currFrames.size());
        point_t origin({0, 0, 1});
        for (int i = 0; i < currFrames.size(); i++) {
            pts[i] = lidar::polarToCartesian2(currFrames[i]);
        }
        points_lock.lock();
        last_points = pts;
        points_lock.unlock();
        lidar_initialized = true;
    } 
}


} // namespace lidar

