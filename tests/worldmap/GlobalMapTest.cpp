#include "../../src/worldmap/GlobalMap.h"

#include <iostream>

#include <catch2/catch.hpp>

namespace
{
double rand(double low, double high)
{
	return low + (::rand() / (RAND_MAX / (high - low))); // NOLINT(cert-msc50-cpp)
}

transform_t randTransform(double centerX, double centerY)
{
	// move back to the origin, rotate around the origin, then move back.
	transform_t toOrigin = toTransformRotateFirst(centerX, centerY, 0);
	transform_t fromOrigin = toTransformRotateFirst(-centerX, -centerY, 0);
	transform_t trf =
		toTransformRotateFirst(rand(-0.1, 0.1), rand(-0.1, 0.1), rand(-M_PI / 32, M_PI / 32));
	return fromOrigin * trf * toOrigin;
}

double calculateMSE(const points_t &p1, const points_t &p2)
{
	double mse = 0;
	for (int i = 0; i < p1.size(); i++)
	{
		point_t truth = p1[i];
		point_t p = p2[i];
		//		std::cout << p(0) << ", " << p(1) << std::endl;
		mse += pow((p - truth).norm(), 2);
	}
	mse /= p1.size();

	return mse;
}
} // namespace

TEST_CASE("Global Map")
{
	srand(time(nullptr)); // NOLINT(cert-msc51-cpp)

	GlobalMap map;
	points_t allPoints;
	// create 5 groups of points for aligning with each other, all in same area
	for (int i = 0; i < 5; i++)
	{
		points_t sample;
		// last sample should be in axes reference frame, so we can check performance easily
		transform_t trf = i < 4 ? randTransform(0, 0) : transform_t::Identity();
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
	double mse = calculateMSE(allPoints, mapPoints);

	std::cout << "Global Map MSE Full Overlap: " << mse << std::endl;
	REQUIRE(mse == Approx(0).margin(0.5));
}

TEST_CASE("Global Map 0.5 Overlap")
{
	srand(time(nullptr)); // NOLINT(cert-msc51-cpp)
	// we'll be getting points along the sin function
	std::function<double(double)> func = [](double x) { return sin(x); };

	GlobalMap map;
	points_t truths;
	// create 5 groups of points for aligning, each partially overlapping with the last
	for (int i = -2; i <= 2; i++)
	{
		// the area is 3 wide, and the center moves right by 1.5 each time
		double center = i * 1.5;
		double min = center - 1.5;
		double max = center + 1.5;
		points_t sample;
		// last sample should be in axes reference frame, so we can check performance easily
		transform_t trf =
			i != 2 ? randTransform(center, func(center)) : transform_t::Identity();
		for (int j = 0; j < 100; j++)
		{
			double x = rand(min, max);
			double y = func(x);
			point_t p = {x, y, 1};
			truths.push_back(p);
			sample.push_back(trf * p);
		}

		map.addPoints(transform_t::Identity(), sample, 0.4);
	}

	points_t mapPoints = map.getPoints();
	double mse = calculateMSE(truths, mapPoints);

	std::cout << "Global Map MSE Partial Overlap: " << mse << std::endl;
	REQUIRE(mse == Approx(0).margin(0.4));
}
