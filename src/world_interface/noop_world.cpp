#include "world_interface.h"

#include <unistd.h>

void world_interface_init() {
}

double setCmdVel(double /*dtheta*/, double /*dx*/) {
	return 1.0;
}

DataPoint<points_t> readLidarScan() {
	return points_t{};
}

DataPoint<points_t> readLandmarks() {
	return points_t{};
}

DataPoint<transform_t> readGPS() {
	return toTransform({0, 0, 0});
}

point_t gpsToMeters(double lon, double lat) {
	return {lon, lat, 1.0};
}

DataPoint<transform_t> readOdom() {
	return toTransform({0, 0, 0});
}

URCLeg getLeg(int /*id*/) {
	return URCLeg{-1, -1, {0., 0., 0.}};
}
