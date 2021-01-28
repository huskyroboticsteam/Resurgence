#include "../../src/worldmap/GlobalMap.h"

#include <iostream>

#include <catch2/catch.hpp>

namespace
{
double rand(double low, double high)
{
	return low + (::rand() / (RAND_MAX / (high - low))); // NOLINT(cert-msc50-cpp)
}

transform_t randTransform() {
	return toTransformRotateFirst(rand(-0.3,0.3),
								  rand(-0.3,0.3),
								  rand(-M_PI/32, M_PI/32));
}
} // namespace

TEST_CASE("Global Map") {
	srand(time(nullptr)); // NOLINT(cert-msc51-cpp)

	GlobalMap map;
	points_t allPoints;
	for (int i = 0; i < 5; i++) {
		points_t sample;
		// the last sample should be in axes reference frame, so we can check performance easily
		transform_t trf = i < 4 ? randTransform() : transform_t::Identity();
		for (int j = 0; j < 150; j++)
		{
			double x1 = rand(-3, 3);
			double y1 = pow(x1, 3);
			point_t p = {x1, y1, 1};
			allPoints.push_back(p);
			sample.push_back(trf * p);
		}
		map.addPoints(transform_t::Identity(), sample, 1);
	}

	points_t mapPoints = map.getPoints();
	double mse = 0;
	for (int i = 0; i < mapPoints.size(); i++) {
		point_t truth = allPoints[i];
		point_t p = mapPoints[i];
//		std::cout << p(0) << ", " << p(1) << std::endl;
		mse += pow((p - truth).norm(), 2);
	}
	mse /= mapPoints.size();

	std::cout << "Global Map MSE: " << mse << std::endl;
	REQUIRE(mse == Approx(0).margin(0.5));
}
