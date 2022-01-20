#include "gps_util.h"

#include <cmath>

constexpr double PI = M_PI;

/**
 * Reference links:
 * https://en.wikipedia.org/wiki/Geodetic_datum#Earth_reference_ellipsoid
 * https://en.wikipedia.org/wiki/World_Geodetic_System#1984_version
 * https://en.wikipedia.org/wiki/Longitude#Length_of_a_degree_of_longitude
 * https://en.wikipedia.org/wiki/Latitude#Meridian_distance_on_the_ellipsoid
 */

// source: https://en.wikipedia.org/wiki/World_Geodetic_System#1984_version
const GPSDatum GPSDatum::WGS84(6378137.0, 6356752.314245);

GPSDatum::GPSDatum(double a, double b) : a(a), b(b) {}

double GPSDatum::getA() const {
	return a;
}

double GPSDatum::getB() const {
	return b;
}

double GPSDatum::getFlattening() const {
	return 1 - b / a;
}

double GPSDatum::getSquareEccentricity() const {
	return 1 - (b * b) / (a * a);
}

GPSToMetersConverter::GPSToMetersConverter(const GPSDatum& datum, const gpscoords_t& origin)
	: datum(datum), origin(origin) {
	double phi = PI * origin.lat / 180.0;
	double cosPhi = std::cos(phi);
	double sinPhi = std::sin(phi);

	double a = datum.getA();
	double eSq = datum.getSquareEccentricity();
	double oneMinusESqSinSqPhi = 1 - eSq * std::pow(sinPhi, 2);

	metersPerDegLon = (PI * a * cosPhi) / (180.0 * std::sqrt(oneMinusESqSinSqPhi));
	metersPerDegLat = (PI * a * (1 - eSq)) / (180.0 * std::pow(oneMinusESqSinSqPhi, 1.5));
}

point_t GPSToMetersConverter::gpsToMeters(const gpscoords_t& coords) const {
	double degDiffLat = coords.lat - origin.lat;
	double degDiffLon = coords.lon - origin.lon;
	// +lat is +x, +lon is -y
	return {degDiffLat * metersPerDegLat, -degDiffLon * metersPerDegLon, 1};
}

gpscoords_t GPSToMetersConverter::metersToGPS(const point_t& pos) const {
	// +x is +lat, +y is -lon
	double degDiffLat = pos(0) / metersPerDegLat;
	double degDiffLon = -pos(1) / metersPerDegLon;
	return {origin.lat + degDiffLat, origin.lon + degDiffLon};
}

double GPSToMetersConverter::getMetersPerDegLon() const {
	return metersPerDegLon;
}

double GPSToMetersConverter::getMetersPerDegLat() const {
	return metersPerDegLat;
}
