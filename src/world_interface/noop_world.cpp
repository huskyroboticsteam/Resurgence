#include "world_interface.h"

#include <unistd.h>

void world_interface_init() {}

double setCmdVel(double /*dtheta*/, double /*dx*/) {
	return 1.0;
}

std::pair<double, double> getCmdVel() {
	return {0, 0};
}

DataPoint<points_t> readLidarScan() {
	return points_t{};
}

landmarks_t readLandmarks() {
	return landmarks_t{};
}

DataPoint<gpscoords_t> gps::readGPSCoords() {
	return {};
}

DataPoint<double> readHeading() {
	return {};
}

DataPoint<pose_t> readVisualOdomVel() {
	return DataPoint<pose_t>{};
}

DataPoint<transform_t> readOdom() {
	return toTransform({0, 0, 0});
}

URCLeg getLeg(int /*id*/) {
	return URCLeg{-1, -1, {0., 0., 0.}};
}

void setMotorPWM(const std::string& motor, double normalizedPWM) {}

void setMotorPos(const std::string& motor, int32_t targetPos) {}

void setIndicator(indication_t signal) {}
