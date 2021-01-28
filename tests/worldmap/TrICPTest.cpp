#include "../../src/worldmap/TrICP.h"

#include <iostream>

#include <catch2/catch.hpp>

#include "../../src/worldmap/GlobalMap.h"

namespace
{
double rand(double low, double high)
{
	return low + (::rand() / (RAND_MAX / (high - low))); // NOLINT(cert-msc50-cpp)
}
} // namespace

TEST_CASE("Trimmed ICP")
{
	points_t map;
	points_t sample;
	points_t truths;
	transform_t trf = toTransformRotateFirst(0.1, -0.25, M_PI / 24);
	srand(time(nullptr)); // NOLINT(cert-msc51-cpp)
	for (int i = 0; i < 150; i++)
	{
		double x1 = rand(-6, 2);
		double y1 = pow(x1, 3);
		map.push_back({x1, y1, 1});

		double x2 = rand(-2, 6);
		double y2 = pow(x2, 3);
		point_t p = {x2, y2, 1};
		truths.push_back(p);
		p = trf * p;
		sample.push_back(p);
	}

	GlobalMap globalMap;
	globalMap.addPoints(transform_t::Identity(), map);

	TrICP icp(25, 0.005, std::bind(&GlobalMap::getClosest, &globalMap, std::placeholders::_1));
	points_t corrected = icp.correct(sample, 0.3);

	double mse = 0;
	for (int i = 0; i < corrected.size(); i++)
	{
		point_t point = corrected[i];
		point_t truth = truths[i];
		mse += pow((point - truth).norm(), 2);
	}

	mse /= corrected.size();

	std::cout << "MSE: " << mse << std::endl;
	REQUIRE(mse <= 0.7); // threshold is arbitrary, in this test the error is usually far lower
}
