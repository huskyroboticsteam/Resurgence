#include "../navtypes.h"
#include "../utils/transform.h"
#include "world_interface.h"

#include <optional>

using namespace navtypes;
using namespace robot::types;

static std::optional<GPSToMetersConverter> converter;

namespace robot {

DataPoint<double> readIMUHeading() {
	return readIMU().transform(
		[](const Eigen::Quaterniond& q) { return util::quatToHeading(q); });
}

DataPoint<point_t> readGPS() {
	DataPoint<gpscoords_t> coords = gps::readGPSCoords();
	if (!coords) {
		return {};
	}

	if (!converter) {
		// just recieved first fix, construct map centered at the first datapoint
		converter.emplace(GPSDatum::WGS84, coords.getData());
	}

	datatime_t time = coords.getTime();
	point_t pos = converter->gpsToMeters(coords.getData());

	return {time, pos};
}

std::optional<point_t> gpsToMeters(const gpscoords_t& coord) {
	if (gpsHasFix()) {
		return converter->gpsToMeters(coord);
	} else {
		return {};
	}
}

bool gpsHasFix() {
	if (!converter.has_value()) {
		readGPS();
	}
	return converter.has_value();
}

} // namespace robot
