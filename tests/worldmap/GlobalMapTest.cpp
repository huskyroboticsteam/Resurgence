#include "../../src/worldmap/GlobalMap.h"
#include "../../src/navtypes.h"
#include "../../src/utils/transform.h"

#include <iostream>
#include <cmath>
#include <cfloat>

#include <catch2/catch.hpp>

using namespace navtypes;
using namespace util;

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
	REQUIRE(p2.size() >= p1.size());
	for (size_t i = 0; i < p1.size(); i++)
	{
		point_t truth = p1[i];
		point_t p = p2[i];
		mse += pow((p - truth).norm(), 2);
	}
	mse /= p1.size();

	return mse;
}
} // namespace

TEST_CASE("Global Map - ScanStride", "[GlobalMap]")
{
	srand(time(nullptr)); // NOLINT(cert-msc51-cpp)

	for (int i = 1; i <= 3; i++) {
		GlobalMap map(1000, i);
		points_t points;
		for (int j = 0; j < i * 1000; j++) {
			point_t point = {rand(-100, 100), rand(-100, 100), 1};
			points.push_back(point);
		}
		map.addPoints(transform_t::Identity(), points, 0);

		REQUIRE(map.size() == 1000);
		REQUIRE(map.getPoints().size() == 1000);
	}
}

TEST_CASE("Global Map - GetClosest", "[GlobalMap]")
{
	srand(time(nullptr)); // NOLINT(cert-msc51-cpp)

	GlobalMap map(1000);
	points_t points;

	constexpr int areaMin = -5;
	constexpr int areaMax = 5;

	for (int i = 0; i < 100; i++)
	{
		double x = rand(areaMin, areaMax);
		double y = rand(areaMin, areaMax);
		point_t p = {x, y, 1};
		points.push_back(p);
	}
	map.addPoints(transform_t::Identity(), points, 0);

	for (int i = 0; i < 500; i++) {
		double x = rand(areaMin, areaMax);
		double y = rand(areaMin, areaMax);
		point_t point = {x, y, 1};

		// manually compute the nearest neighbor to check against
		point_t closest = {0, 0, 0};
		double minDist = std::numeric_limits<double>::infinity();
		for (const point_t &p : points)
		{
			double dist = (p - point).topRows<2>().norm();
			if (dist < minDist)
			{
				minDist = dist;
				closest = p;
			}
		}

		point_t closestMap = map.getClosest(point);
		REQUIRE(closest == closestMap);
		REQUIRE(map.hasPointWithin(point, minDist));

		REQUIRE(map.hasPointWithin(point, std::nextafter(minDist, DBL_MAX)));
		REQUIRE_FALSE(map.hasPointWithin(point, std::nextafter(minDist, 0)));

		REQUIRE(map.hasPointWithin(point, minDist * 2));
		REQUIRE_FALSE(map.hasPointWithin(point, minDist / 2.0));
	}
}

TEST_CASE("Global Map", "[GlobalMap]")
{
	srand(time(nullptr)); // NOLINT(cert-msc51-cpp)

	GlobalMap map(1000);
	points_t allPoints;
	// create 5 groups of points for aligning with each other, all in same area
	for (int i = 0; i < 5; i++)
	{
		points_t sample;
		// first sample should be in axes reference frame, so we can check performance easily
		transform_t trf = i > 0 ? randTransform(0, 0) : transform_t::Identity();
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

	// we need to sort because iteration order is not consistent
	auto comp = [](const point_t &a, const point_t &b) { return a(0) < b(0); };
	std::sort(allPoints.begin(), allPoints.end(), comp);
	std::sort(mapPoints.begin(), mapPoints.end(), comp);

	double mse = calculateMSE(allPoints, mapPoints);

	std::cout << "Global Map MSE Full Overlap: " << mse << std::endl;
	REQUIRE(mse == Approx(0).margin(0.5));
}

TEST_CASE("Global Map 0.5 Overlap", "[GlobalMap]")
{
	srand(time(nullptr)); // NOLINT(cert-msc51-cpp)
	// we'll be getting points along the sin function
	std::function<double(double)> func = [](double x) { return sin(x); };

	GlobalMap map(1000);
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

	// we need to sort because iteration order is not consistent
	auto comp = [](const point_t &a, const point_t &b) { return a(0) < b(0); };
	std::sort(truths.begin(), truths.end(), comp);
	std::sort(mapPoints.begin(), mapPoints.end(), comp);

	double mse = calculateMSE(truths, mapPoints);

	std::cout << "Global Map MSE Partial Overlap: " << mse << std::endl;
	REQUIRE(mse == Approx(0).margin(0.4));
}
