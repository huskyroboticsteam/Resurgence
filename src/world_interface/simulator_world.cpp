#include "../simulator/world.h"
#include "world_interface.h"

#include <iostream>
#include <unistd.h>

World world;
std::pair<double, double> cmdVel(0, 0);

void world_interface_init() {
	world.addURCObstacles();
	world.start();
	// Sleep to avoid situation where we're trying to launch two SFML windows
	// simultaneously (which seems to sometimes cause deadlock??)
	usleep(300 * 1000);
}

double setCmdVel(double dtheta, double dx) {
	world.setCmdVel(dtheta, dx);
	cmdVel = {dtheta, dx};
	return 1.0;
}

std::pair<double, double> getCmdVel() {
	return cmdVel;
}

DataPoint<points_t> readLidarScan() {
	return world.readLidar();
}

landmarks_t readLandmarks() {
	points_t lm = world.readLandmarks();
	datatime_t now = dataclock::now();
	landmarks_t ret;
	for (const point_t &point : lm) {
		if (point(2) != 0) {
			ret.emplace_back(now, point);
		} else {
			ret.emplace_back();
		}
	}
	return ret;
}

DataPoint<transform_t> readGPS() {
	return world.readGPS();
}

DataPoint<pose_t> readVisualOdomVel() {
	return DataPoint<pose_t>{};
}

point_t gpsToMeters(double lon, double lat) {
	return {lon, lat, 1.0};
}

DataPoint<transform_t> readOdom() {
	return world.readOdom();
}

URCLeg getLeg(int index) {
	return world.getLeg(index);
}

// not supported in 2D sim
void setMotorPWM(const std::string& motor, double normalizedPWM) {}

// not supported in 2D sim
void setMotorPos(const std::string& motor, int32_t targetPos) {}

void setIndicator(indication_t signal) {
	std::string signals[] = {"off", "autonomous", "teleop", "arrivedAtDest"};
	auto idx = static_cast<int>(signal);
	std::cout << "Setting indicator: " << signals[idx] << std::endl;
}
