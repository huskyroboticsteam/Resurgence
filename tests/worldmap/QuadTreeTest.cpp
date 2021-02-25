#include <iostream>

#include "../../src/worldmap/QuadTree.h"

#include <catch2/catch.hpp>

namespace {
double randNum(double low, double high) {
	return low + (rand() / (RAND_MAX / (high - low))); // NOLINT(cert-msc50-cpp)
}

point_t randPoint(double size) {
	return {randNum(-size/2,size/2), randNum(-size/2, size/2), 1};
}
}

TEST_CASE("QuadTree") {
	srand(time(nullptr));
	QuadTree tree(100);

	REQUIRE(tree.add({0,0,1}));
	REQUIRE(tree.add({0,1,1}));
	REQUIRE(tree.add({0,3,1}));

	REQUIRE(tree.getClosestWithin(point_t(1,1,1), 3) == point_t(0,1,1));
}
