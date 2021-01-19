#include "../../src/worldmap/TrICP.h"

#include <iostream>

#include <catch2/catch.hpp>

namespace {
double rand(double low, double high) {
	return low + (::rand() / (RAND_MAX / (high - low)));
}
}

TEST_CASE("Trimmed ICP") {
	points_t map;
	points_t sample;
	transform_t trf = toTransformRotateFirst(0.1, -0.25, M_PI/24);
	srand(10);
	for (int i = 0; i < 50; i++) {
		double x1 = rand(-3, 2);
		double y1 = pow(x1, 2);
		map.push_back({x1, y1, 1});

		double x2 = rand(-2, 3);
		double y2 = pow(x2, 2);
		point_t p = {x2, y2, 1};
		p = trf * p;
		std::cout << p(0) << "," << p(1) << std::endl;
		sample.push_back(p);
	}

	std::cout << "\n\n\n\n" << std::endl;

	GlobalMap globalMap;
	globalMap.addPoints(transform_t::Identity(), map);

	TrICP icp(globalMap, 0.6, 15, 0.01);
	points_t corrected = icp.correct(sample);

	double mse = 0;
	for (const point_t &point : corrected) {
		std::cout << point(0) <<","<<point(1) << std::endl;
		double y = pow(point(0), 2);
		point_t truth = {point(0), y, 1};
		mse += pow((point - truth).norm(), 2);
	}

	mse /= corrected.size();

	std::cout << "MSE: " << mse << std::endl;
	REQUIRE(mse <= 0.7);
}
