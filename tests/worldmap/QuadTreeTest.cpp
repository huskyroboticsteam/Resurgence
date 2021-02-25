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

TEST_CASE("QuadTree - AddPoints", "[QuadTree]") {
	srand(time(nullptr)); // NOLINT(cert-msc51-cpp)
	QuadTree tree(100);

	for (int i = 0; i < 50; i++) {
		const point_t &point = randPoint(100);
		REQUIRE(tree.getSize() == i);
		REQUIRE(tree.add(point));
		REQUIRE(tree.getSize() == i+1);
	}
}
