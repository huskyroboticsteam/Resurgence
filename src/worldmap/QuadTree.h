#pragma once

#include <memory>

#include "../simulator/utils.h"

class QuadTree
{
public:
	explicit QuadTree(double width, int nodeCapacity = 16);
	QuadTree(point_t center, double width, int nodeCapacity = 16);

	size_t getSize() const;
	bool empty() const;
	points_t getAllPoints() const;
	bool add(const point_t &point);
	bool remove(const point_t &point);
	point_t getClosestWithin(const point_t &point, double areaSize) const;
	points_t getPointsWithin(const point_t &point, double areaSize) const;

private:
	std::shared_ptr<QuadTree> children[4];
	point_t center;
	double width;
	points_t points;
	int nodeCapacity;
	size_t size;

	void subdivide();
	bool hasChildren() const;
	void getAllPoints(const QuadTree &tree, points_t &allPoints) const;
};
