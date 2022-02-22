#include "../navtypes.h"
#include "world_interface.h"

#include <unistd.h>

using namespace navtypes;
using namespace robot::types;

namespace robot {

const WorldInterface WORLD_INTERFACE = WorldInterface::noop;

void world_interface_init() {}

DataPoint<points_t> readLidarScan() {
	return points_t{};
}

landmarks_t readLandmarks() {
	return landmarks_t{};
}

DataPoint<double> readIMUHeading() {
	return {};
}

DataPoint<pose_t> getTruePose() {
	return {};
}

DataPoint<pose_t> readVisualOdomVel() {
	return DataPoint<pose_t>{};
}

URCLeg getLeg(int /*id*/) {
	return URCLeg{-1, -1, {0., 0., 0.}};
}

void setMotorPWM(const std::string& motor, double normalizedPWM) {}

void setMotorPos(const std::string& motor, int32_t targetPos) {}

void setIndicator(indication_t signal) {}

} // namespace robot

DataPoint<gpscoords_t> gps::readGPSCoords() {
	return {};
}
