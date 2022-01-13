
#include "read_hokuyo_lidar.h"

#include "../simulator/utils.h"
#include "../world_interface/world_interface.h"
#include "PointCloudProcessing.h"
#include "PointGenerator.h"
#include "URGLidar.h"

#include <atomic>
#include <mutex>
#include <thread>

// This website may be helpful
// https://sourceforge.net/p/urgnetwork/wiki/top_en/

constexpr double LIDAR_MIN_RANGE = 0.15;

std::atomic<bool> lidar_initialized(false);
std::atomic<datatime_t> lidar_time(datatime_t{});
URGLidar urg_lidar;
std::thread lidar_thread;
points_t last_points = {};
std::mutex points_lock;

namespace lidar {

void readLidarLoop() {
	while (true) {
		if (!urg_lidar.createFrame()) {
			perror("failed to create frame");
			continue;
		}

		std::vector<Polar2D> polarPts = urg_lidar.getLastFrame();
		lidar_time = dataclock::now();
		std::vector<point_t> pts(polarPts.size());
		point_t origin({0, 0, 1});
		for (size_t i = 0; i < polarPts.size(); i++) {
			pts[i] = lidar::polarToCartesian2(polarPts[i]);
			if ((pts[i] - origin).norm() < LIDAR_MIN_RANGE) {
				// We're inside lidar min range, so this is garbage data.
				// We don't want to include this hit because otherwise the robot
				// will think it's in collision with an obstacle.
				pts[i](2) = 0.0;
			}
		}
		points_lock.lock();
		last_points = pts;
		points_lock.unlock();
		lidar_initialized = true;
	}
}

bool initializeLidar() {
	if (!lidar_initialized) {
		if (!urg_lidar.open()) {
			perror("failed to open lidar");
		} else {
			lidar_thread = std::thread(&readLidarLoop);
			// we won't mark the lidar as initialized until the first scan has been read
		}
	}
	return lidar_initialized;
}

DataPoint<points_t> readLidar() {
	if (lidar_initialized) {
		points_t pts;
		points_lock.lock();
		pts = last_points;
		points_lock.unlock();
		datatime_t time = lidar_time;
		return {time, pts};
	} else {
		return {};
	}
}
} // namespace lidar
