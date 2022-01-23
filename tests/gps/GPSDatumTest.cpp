#include "../../src/gps/gps_util.h"

#include <catch2/catch.hpp>

TEST_CASE("Test WGS84 Datum") {
	GPSDatum datum = GPSDatum::WGS84;

	// numbers from here: https://en.wikipedia.org/wiki/World_Geodetic_System
	CHECK(datum.getA() == Approx(6378137.0));
	CHECK(datum.getB() == Approx(6356752.314245));
	CHECK(datum.getFlattening() == Approx(1.0 / 298.257223563));
	CHECK(datum.getSquareEccentricity() == Approx(0.00669437999014));
}
