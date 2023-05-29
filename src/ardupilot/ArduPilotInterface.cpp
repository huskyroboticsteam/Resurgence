#include "ArduPilotInterface.h"

#include <memory>

using namespace robot::types;
using namespace navtypes;
using namespace std::chrono_literals;

namespace {

std::unique_ptr<net::ardupilot::ArduPilotProtocol> ardupilot_protocol;

} // namespace

namespace ardupilot {
void initArduPilotProtocol(net::websocket::SingleClientWSServer& websocketServer) {
	ardupilot_protocol = std::make_unique<net::ardupilot::ArduPilotProtocol>(websocketServer);
}
} // namespace ardupilot

namespace gps {
DataPoint<gpscoords_t> readGPSCoords() {
	return ardupilot_protocol->getGPS();
}
} // namespace gps

namespace robot {
DataPoint<double> readIMUHeading() {
	auto imu_heading = ardupilot_protocol->getIMU();
	if (imu_heading.isValid()) {
		return DataPoint<double>(imu_heading.getTime(), imu_heading.getData().yaw);
	}
	return DataPoint<double>();
}
} // namespace robot
