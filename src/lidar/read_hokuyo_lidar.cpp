
#include "PointCloudProcessing.h"
#include "PointGenerator.h"
#include "URGLidar.h"
#include <atomic>
#include <thread>
#include <mutex>
#include "../simulator/world_interface.h"
#include "../simulator/utils.h"

// This website may be helpful
// https://sourceforge.net/p/urgnetwork/wiki/top_en/

constexpr double LIDAR_MIN_RANGE = 0.015;

std::atomic<bool> lidar_initialized(false);
URGLidar urg_lidar;
std::thread lidar_thread;
points_t last_points = {};
std::mutex points_lock;

void readLidarLoop(){
	while(true){
		if(lidar_initialized){
			if (!urg_lidar.createFrame())
			{
				// TODO: should we be using log() here instead?
				perror("failed to create frame");
				points_lock.lock();
				last_points = {};
				points_lock.unlock();
			}

			std::vector<Polar2D> polarPts = urg_lidar.getLastFrame();
			std::vector<point_t> pts(polarPts.size());
			point_t origin({0,0,1});
			for (int i = 0; i < polarPts.size(); i++)
			{
				pts[i] = lidar::polarToCartesian2(polarPts[i]);
				if ((pts[i] - origin).norm() < LIDAR_MIN_RANGE)
				{
					// We're inside lidar min range, so this is garbage data.
					// We don't want to include this hit because otherwise the robot
					// will think it's in collision with an obstacle.
					pts[i](2) = 0.0;
				}
			}
			points_lock.lock();
			last_points = pts;
			points_lock.unlock();
		}
	}
}

points_t readLidarScan() {
	if (!lidar_initialized) {
		if (!urg_lidar.open())
		{
			// TODO: should we be using log() here instead?
			perror("failed to open lidar");
			return {};
		}
		else
		{
			lidar_thread = std::thread(&readLidarLoop);
			lidar_initialized = true;
		}
	}

	points_t pts;
	points_lock.lock();
	pts = last_points;
	points_lock.unlock();
	return pts;
}
