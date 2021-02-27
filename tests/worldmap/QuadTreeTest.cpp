#include "../../src/worldmap/QuadTree.h"

#include <iostream>

#include <catch2/catch.hpp>

namespace
{
double randNum(double low, double high)
{
	return low + (rand() / (RAND_MAX / (high - low))); // NOLINT(cert-msc50-cpp)
}

point_t randPoint(double size)
{
	return {randNum(-size / 2, size / 2), randNum(-size / 2, size / 2), 1};
}
} // namespace

TEST_CASE("QuadTree - AddPoints", "[QuadTree]")
{
	srand(time(nullptr)); // NOLINT(cert-msc51-cpp)
	QuadTree tree(100);

	REQUIRE(tree.empty());

	for (int i = 0; i < 50; i++)
	{
		const point_t &point = randPoint(100);
		REQUIRE(tree.getSize() == i);
		REQUIRE(tree.add(point));
		REQUIRE(tree.getSize() == i + 1);
	}

	REQUIRE_FALSE(tree.empty());
}

TEST_CASE("QuadTree - RemovePoints", "[QuadTree]")
{
	srand(time(nullptr)); // NOLINT(cert-msc51-cpp)
	QuadTree tree(100);

	points_t points;

	for (int i = 0; i < 50; i++)
	{
		const point_t &point = randPoint(100);
		points.push_back(point);
		REQUIRE(tree.getSize() == i);
		REQUIRE(tree.add(point));
		REQUIRE(tree.getSize() == i + 1);
	}

	int size = tree.getSize();
	for (const point_t &point : points)
	{
		REQUIRE(tree.getSize() == size);
		REQUIRE(tree.remove(point));
		REQUIRE(tree.getSize() == --size);
	}
	REQUIRE(tree.getSize() == 0);
	REQUIRE(tree.empty());
}

TEST_CASE("QuadTree - RemoveNonexistentPoints", "[QuadTree]")
{
	srand(time(nullptr)); // NOLINT(cert-msc51-cpp)
	QuadTree tree(100, 4);

	for (int i = 0; i < 50; i++)
	{
		// make the size smaller, so we can guarantee that some points are not in the tree
		tree.add(randPoint(5));
	}

	auto size = tree.getSize();
	REQUIRE_FALSE(tree.remove({53, 1, 1}));
	REQUIRE_FALSE(tree.remove({0, 0, 0}));
	REQUIRE_FALSE(tree.remove({-32, 63, 1}));
	REQUIRE_FALSE(tree.remove({-32, -84, 1}));
	REQUIRE_FALSE(tree.remove({99, -13, 1}));
	REQUIRE(size == tree.getSize());
}

TEST_CASE("QuadTree - GetAllPoints", "[QuadTree]")
{
	srand(time(nullptr)); // NOLINT(cert-msc51-cpp)
	QuadTree tree(100, 4);

	points_t points;

	for (int i = 0; i < 50; i++)
	{
		const point_t &point = randPoint(100);
		points.push_back(point);
		tree.add(point);
	}

	REQUIRE(points.size() == tree.getSize());

	// assert that all points are in this list
	points_t treePoints = tree.getAllPoints();
	REQUIRE(points.size() == treePoints.size());
	for (const point_t &point : points)
	{
		auto itr = std::find(treePoints.begin(), treePoints.end(), point);
		REQUIRE(itr != treePoints.end());
		REQUIRE(point == *itr);
	}

	// assert that modifying this list doesn't modify the tree
	auto prevSize = treePoints.size();
	point_t erased = *treePoints.begin();
	treePoints.erase(treePoints.begin());
	// not a catch2 assert since this is just a sanity check
	assert(prevSize == treePoints.size() + 1);

	REQUIRE(tree.getAllPoints().size() == prevSize);
	REQUIRE(tree.getClosestWithin(erased, 1) == erased);

	// verify that mutating the objects in the list doesn't modify the tree
	treePoints = tree.getAllPoints();
	REQUIRE(tree.getClosestWithin(treePoints[0], 0.5) == treePoints[0]);
	treePoints[0](0) = treePoints[0](0) + 0.01;
	REQUIRE(tree.getClosestWithin(treePoints[0], 0.5) != treePoints[0]);
}

TEST_CASE("QuadTree - GetPointsWithin", "[QuadTree]")
{
	srand(time(nullptr)); // NOLINT(cert-msc51-cpp)
	QuadTree tree(100, 4);

	// create two bunches: bottom-left and top-right
	points_t left, right;

	for (int i = 0; i < 50; i++)
	{
		point_t p = randPoint(10) - point_t(20, 20, 0);
		left.push_back(p);
		tree.add(p);
	}

	for (int i = 0; i < 50; i++)
	{
		point_t p = randPoint(10) + point_t(20, 20, 0);
		right.push_back(p);
		tree.add(p);
	}

	REQUIRE(tree.getSize() == (left.size() + right.size()));

	points_t closestToLeft = tree.getPointsWithin(point_t(-20, -20, 1), 25);
	points_t closestToRight = tree.getPointsWithin(point_t(20, 20, 1), 25);

	// verify that we retrieved all the points in the bottom-left bunch
	REQUIRE(closestToLeft.size() == left.size());
	for (const point_t &point : left)
	{
		auto itr = std::find(closestToLeft.begin(), closestToLeft.end(), point);
		REQUIRE(itr != closestToLeft.end());
		REQUIRE(point == *itr);
	}

	// verify that we retrieved all the points in the top-right bunch
	REQUIRE(closestToRight.size() == right.size());
	for (const point_t &point : right)
	{
		auto itr = std::find(closestToRight.begin(), closestToRight.end(), point);
		REQUIRE(itr != closestToRight.end());
		REQUIRE(point == *itr);
	}
}

TEST_CASE("QuadTree - GetClosestPoint", "[QuadTree]")
{
	srand(time(nullptr)); // NOLINT(cert-msc51-cpp)
	QuadTree tree(100);

	for (int i = 0; i < 100; i++)
	{
		tree.add(randPoint(100));
	}

	points_t points = tree.getAllPoints();

	for (const point_t &point : points)
	{
		point_t closestTree = tree.getClosest(point);
		point_t closest = {0, 0, 0};
		double minDist = std::numeric_limits<double>::infinity();
		for (const point_t &p : points)
		{
			double dist = (p - point).norm();
			if (dist < minDist)
			{
				minDist = dist;
				closest = p;
			}
		}
		REQUIRE(closest == closestTree);
	}
}

TEST_CASE("QuadTree - TestBoundary", "[QuadTree]")
{
	QuadTree tree(100);

	// test that adding things in the boundary work
	REQUIRE(tree.add({50, 50, 1}));
	REQUIRE_FALSE(tree.add({100, 100, 1}));

	REQUIRE(tree.add({-50, -50, 1}));
	REQUIRE_FALSE(tree.add({-100, -100, 1}));

	REQUIRE(tree.getClosest({50, 50, 1}) == point_t(50, 50, 1));
	REQUIRE(tree.getClosest({-50, -50, 1}) == point_t(-50, -50, 1));

	REQUIRE(tree.getClosest({1, 1, 1}) == point_t(50, 50, 1));
	REQUIRE(tree.getClosest({-1, -1, 1}) == point_t(-50, -50, 1));
}

TEST_CASE("QuadTree - EmptyTree", "[QuadTree]")
{
	// test behavior of empty graph
	QuadTree tree(100);

	REQUIRE(tree.empty());
	REQUIRE(tree.getSize() == 0);
	REQUIRE(tree.getClosest({0, 0, 1}) == point_t(0, 0, 0));
	REQUIRE(tree.getClosestWithin({0, 0, 1}, 15) == point_t(0, 0, 0));
	points_t points = tree.getPointsWithin(point_t(0, 0, 1), 15);
	REQUIRE(points.empty());
}
