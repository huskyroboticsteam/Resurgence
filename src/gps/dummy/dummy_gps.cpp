#include "../../world_interface/data.h"

using robot::types::DataPoint;

namespace gps {
DataPoint<navtypes::gpscoords_t> readGPSCoords() {
	return {};
}

DataPoint<double> readIMUHeading() {
	return {};
}
}