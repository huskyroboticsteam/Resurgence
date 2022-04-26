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

constexpr uint32_t RPLIDAR_BAUDRATE = 115200;
const double MM_TO_M = 1000;

std::atomic<bool> lidar_initialized(false);
std::atomic<datatime_t> lidar_time(datatime_t{});
RPLidar rp_lidar("/dev/ttyUSB0", RPLIDAR_BAUDRATE);
std::thread lidar_thread;
DataPoint<points_t> last_points = {};
std::mutex points_lock;

namespace lidar {

/**
 * @brief Startsup RPLidar with default settings
 */
bool initializeLidar() {
    if (!lidar_initialized) {
        if (!rp_lidar.checkHealth()) {
            perror("failed to connect to rp lidar");
        } else {
            rp_lidar.setMaxDistance(16.0);
            points_lock.lock();
            lidar_thread = std::thread(&readLidarLoop);
            lidar_initialized = true;
            points_lock.unlock();
        }
    }
    return lidar_initialized;
}

/**
 * @brief RP Lidar continuously scans environment, updates current timeframe with lidar data
 */
void readLidarLoop() {
    using namespace std::chrono;
    std::vector<Polar2D> polarPts;    
    while (true) {
        auto scan = rp_lidar.poll();
        if (auto scan = rp_lidar.poll()) {
            std::vector<point_t> pts;
            double dtheta = (scan.value().angle_max-scan.value().angle_min)/(scan.value().ranges.size()-1);
            for (unsigned long i = 0; i < scan.value().ranges.size(); i++) {
                double rad = dtheta*i;
                double dist = scan.value().ranges[i] * MM_TO_M;

                Polar2D frame{dist, rad};
                pts.push_back(lidar::polarToCartesian2(frame));
            }
        
            points_lock.lock();
            datatime_t time = lidar_time;
            last_points = {time, pts};
            points_lock.unlock();
        } else {
            perror("failed to get frame");
            points_lock.lock();
            datatime_t time = lidar_time;
            last_points = {};
            points_lock.unlock();
            continue;
        }
    } 
}

/**
 * @brief Provides RP Lidar data at current timeframe
 */
DataPoint<points_t> readLidar() {
    if (lidar_initialized) {
        DataPoint<points_t> pts;
        points_lock.lock();
        pts = last_points;
        points_lock.unlock();
        return pts;
    }
    return {};
}


} // namespace lidar

