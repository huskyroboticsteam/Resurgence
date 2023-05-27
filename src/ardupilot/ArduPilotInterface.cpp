#include "ArduPilotProtocol.h"

using namespace robot::types;
using namespace navtypes;
using namespace std::chrono_literals;

namespace {
net::ardupilot::ArduPilotProtocol ardupilot_protocol;
} // namespace

namespace gps {
DataPoint<gpscoords_t> readGPSCoords() {
    return ardupilot_protocol.getGPS();
}
} // namespace gps

namespace robot {
DataPoint<double> readIMUHeading() {
    return ardupilot_protocol.getIMU().getData().yaw;
}
} // namespace robot

