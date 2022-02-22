#include "../../src/gps/gps_util.h"
#include "../../src/navtypes.h"

#include <random>

#include <catch2/catch.hpp>

using namespace navtypes;

namespace {
gpscoords_t seattle{47.608013, -122.335167};
GPSToMetersConverter converter(GPSDatum::WGS84, seattle);

// numbers from http://www.csgnetwork.com/degreelenllavcalc.html
// both of these numbers are for seattle (with the coords above)
constexpr double M_PER_DEG_LAT = 111182.65767187615;
constexpr double M_PER_DEG_LON = 75188.87;

void assertPointEquals(const point_t& expected, const point_t& actual) {
	CHECK(actual(0) == Approx(expected(0)));
	CHECK(actual(1) == Approx(expected(1)));
	CHECK(actual(2) == expected(2));
}

void assertCoordEquals(const gpscoords_t& expected, const gpscoords_t& actual) {
	CHECK(actual.lat == Approx(expected.lat));
	CHECK(actual.lon == Approx(expected.lon));
}

// only works when coords is near seattle
gpscoords_t offsetGPSByMeters(const gpscoords_t& coords, double x, double y) {
	double degLatPerM = 1.0 / M_PER_DEG_LAT;
	double degLonPerM = 1.0 / M_PER_DEG_LON;
	return {coords.lat + x * degLatPerM, coords.lon - y * degLonPerM};
}
} // namespace

TEST_CASE("Test GPS Linearization") {
	// check for Seattle
	GPSToMetersConverter seattleConverter(GPSDatum::WGS84, seattle);
	CHECK(seattleConverter.getMetersPerDegLat() == Approx(M_PER_DEG_LAT));
	CHECK(seattleConverter.getMetersPerDegLon() == Approx(M_PER_DEG_LON));

	// check for Sydney, Australia
	GPSToMetersConverter sydneyConverter(GPSDatum::WGS84, {-33.865143, 151.209900});
	CHECK(sydneyConverter.getMetersPerDegLat() == Approx(110919.93127813512));
	CHECK(sydneyConverter.getMetersPerDegLon() == Approx(92530.49037782029));
}

TEST_CASE("Test map axes") {
	// assert that +lat = +x
	point_t plusX = converter.gpsToMeters({seattle.lat + 0.1, seattle.lon});
	CHECK(plusX(0) > 0);
	CHECK(plusX(1) == Approx(0));
	CHECK(plusX(2) == 1);

	// assert that -lat = -x
	point_t minusX = converter.gpsToMeters({seattle.lat - 0.1, seattle.lon});
	CHECK(minusX(0) < 0);
	CHECK(minusX(1) == Approx(0));
	CHECK(minusX(2) == 1);

	// assert that +lon = -y
	point_t minusY = converter.gpsToMeters({seattle.lat, seattle.lon + 0.1});
	CHECK(minusY(0) == Approx(0));
	CHECK(minusY(1) < 0);
	CHECK(minusY(2) == 1);

	// assert that -lon = +y
	point_t plusY = converter.gpsToMeters({seattle.lat, seattle.lon - 0.1});
	CHECK(plusY(0) == Approx(0));
	CHECK(plusY(1) > 0);
	CHECK(plusY(2) == 1);
}

TEST_CASE("Test GPS to meters") {
	// test specific points
	assertPointEquals({0, 0, 1}, converter.gpsToMeters(seattle));
	assertPointEquals({1, -1, 1}, converter.gpsToMeters(offsetGPSByMeters(seattle, 1, -1)));
	assertPointEquals({1, 1, 1}, converter.gpsToMeters(offsetGPSByMeters(seattle, 1, 1)));
	assertPointEquals({-1, -1, 1}, converter.gpsToMeters(offsetGPSByMeters(seattle, -1, -1)));
	assertPointEquals({12.4, -13.2, 1},
					  converter.gpsToMeters(offsetGPSByMeters(seattle, 12.4, -13.2)));
	assertPointEquals({0.0, -1.01, 1},
					  converter.gpsToMeters(offsetGPSByMeters(seattle, 0.0, -1.01)));

	std::random_device dev;
	std::mt19937 rng(dev());
	// 1km in each direction
	std::uniform_real_distribution<double> dist(-1000, 1000);
	// test a bunch of random numbers
	for (int i = 0; i < 1000; i++) {
		double dx = dist(rng);
		double dy = dist(rng);
		assertPointEquals({dx, dy, 1},
						  converter.gpsToMeters(offsetGPSByMeters(seattle, dx, dy)));
	}
}

TEST_CASE("Test meters to GPS") {
	std::random_device dev;
	std::mt19937 rng(dev());
	// 1km in each direction
	std::uniform_real_distribution<double> dist(-1000, 1000);
	// test a bunch of random numbers
	for (int i = 0; i < 1000; i++) {
		double dx = dist(rng);
		double dy = dist(rng);
		assertCoordEquals(offsetGPSByMeters(seattle, dx, dy),
						  converter.metersToGPS({dx, dy, 1}));
	}
}
