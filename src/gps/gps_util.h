#pragma once

#include "../simulator/utils.h"

struct gpscoords_t {
	double lat;
	double lon;
};

class GPSDatum {
public:
	static const GPSDatum WGS84;

	GPSDatum(double a, double b);

	double getA() const;

	double getB() const;

	double getFlattening() const;

	double getSquareEccentricity() const;

private:
	const double a;
	const double b;
};

class GPSToMetersConverter {
public:
	GPSToMetersConverter(const GPSDatum& datum, const gpscoords_t &origin);

	point_t gpsToMeters(const gpscoords_t &coords) const;

	gpscoords_t metersToGPS(const point_t &pos) const;

	double getMetersPerDegLat() const;
	
	double getMetersPerDegLon() const;

private:
	const GPSDatum datum;
	const gpscoords_t origin;
	double metersPerDegLat;
	double metersPerDegLon;
};
