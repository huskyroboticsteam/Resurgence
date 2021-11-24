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

DataPoint<points_t> readLandmarks() {
	return world.readLandmarks();
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
