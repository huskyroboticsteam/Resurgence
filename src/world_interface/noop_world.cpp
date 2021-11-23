#include "world_interface.h"

#include <unistd.h>

void world_interface_init() {
}

double setCmdVel(double /*dtheta*/, double /*dx*/) {
	return 1.0;
}

std::pair<double, double> getCmdVel() {
    return {0, 0};
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

DataPoint<pose_t> readVisualOdomVel() {
    return DataPoint<pose_t>{};
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

void setMotorPWM(const std::string &motor, double normalizedPWM) {}

void setMotorPos(const std::string &motor, int32_t targetPos) {}
