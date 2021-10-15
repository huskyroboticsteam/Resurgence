#include "../simulator/world.h"
#include "world_interface.h"

#include <unistd.h>

World world;

void world_interface_init() {
	world.addURCObstacles();
	world.start();
	// Sleep to avoid situation where we're trying to launch two SFML windows
	// simultaneously (which seems to sometimes cause deadlock??)
	usleep(300 * 1000);
}

double setCmdVel(double dtheta, double dx) {
	world.setCmdVel(dtheta, dx);
	return 1.0;
}

points_t readLidarScan() {
	return world.readLidar();
}

points_t readLandmarks() {
	return world.readLandmarks();
}

transform_t readGPS() {
	return world.readGPS();
}

point_t gpsToMeters(double lon, double lat) {
	return {lon, lat, 1.0};
}

transform_t readOdom() {
	return world.readOdom();
}

URCLeg getLeg(int index) {
	return world.getLeg(index);
}
