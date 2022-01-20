#pragma once

#include "../simulator/utils.h"

// Represents a GPS coordinate in degrees. Positive is north/east.
struct gpscoords_t {
	// the latitude of the gps coordinate, in degrees
	double lat;
	// the longitude of the gps coordinate, in degrees
	double lon;
};

/**
 * @brief A GPS datum that specifies the reference ellipsoid for use in GPS calculations.
 * This is important to resolve what exact location a gps coordinate specifies.
 */
class GPSDatum {
public:
	static const GPSDatum WGS84;

	/**
	 * @brief Construct a new GPSDatum object, with the specified parameters for the ellipsoid.
	 *
	 * @param a The semi-major axis of the ellipsoid.
	 * @param b The semi-minor axis of the ellipsoid.
	 */
	GPSDatum(double a, double b);

	// Get the semi-major axis of the reference ellipsoid
	double getA() const;

	// Get the semi-minor axis of the reference ellipsoid
	double getB() const;

	// Get the flattening parameter of the reference ellipsoid
	double getFlattening() const;

	// Get the square of the (first) eccentricity of the ellipsoid
	double getSquareEccentricity() const;

private:
	const double a;
	const double b;
};

/**
 * @brief A class used to convert gps coordinates to coordinates on a flat xy-plane, and vice
 * versa. The xy-plane will always be measured in meters. Additionally, the xy-plane will be
 * aligned such that +x = +lat, +y = -lon.
 */
class GPSToMetersConverter {
public:
	/**
	 * @brief Construct a new GPSToMetersConverter object.
	 *
	 * @param datum The datum to use for the GPS calculations.
	 * @param origin The gps coordinates of the origin of the xy-plane.
	 */
	GPSToMetersConverter(const GPSDatum& datum, const gpscoords_t& origin);

	// convert the given gps coordinates to a coordinate on the xy plane, in meters
	point_t gpsToMeters(const gpscoords_t& coords) const;

	// convert a coordinate on the xy-plane to a gps coordinate
	gpscoords_t metersToGPS(const point_t& pos) const;

	// Get the number of meters per degree of latitude.
	// Given the datum and the origin, this is a constant.
	double getMetersPerDegLat() const;

	// Get the number of meters per degree of longitude.
	// Given the datum and the origin, this is a constant.
	double getMetersPerDegLon() const;

private:
	const GPSDatum datum;
	const gpscoords_t origin;
	double metersPerDegLat;
	double metersPerDegLon;
};
