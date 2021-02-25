#pragma once

#include <memory>

#include "../simulator/utils.h"

class QuadTree
{
public:
	explicit QuadTree(double width, int nodeCapacity=16);
	QuadTree(point_t center, double width, int nodeCapacity=16);

	bool add(const point_t &point);
	bool remove(const point_t &point);
	point_t getClosestWithin(const point_t &point, double size);
	points_t getPointsWithin(const point_t &point, double size);

private:
	std::shared_ptr<QuadTree> children[4];
	point_t center;
	double width;
	points_t points;
	int nodeCapacity;

	void subdivide();
	bool hasChildren();
};
