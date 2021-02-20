#pragma once

#include "../simulator/utils.h"

using lochash_t = unsigned long long;

class LinearQuadTree
{
public:
	LinearQuadTree(double width);

	void add(const point_t &point);
	void remove(const point_t &point);
	point_t getClosest(const point_t &point);
	points_t getPointsWithin(const point_t &point, double dist);
private:
	std::vector<lochash_t> hashes;
	double width;

	point_t hashToPoint(lochash_t hash);
	lochash_t pointToHash(const point_t &point);
};
