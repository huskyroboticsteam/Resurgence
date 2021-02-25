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
}
