#include "QuadTree.h"

#include <algorithm>

bool inBounds(const point_t &center, double size, const point_t &point)
{
	point_t diff = (point - center).array().abs();
	double halfSize = size / 2;
	return diff(0) <= halfSize && diff(1) <= halfSize;
}

QuadTree::QuadTree(point_t center, double width, int nodeCapacity)
	: center(std::move(center)), width(width), points({}), nodeCapacity(nodeCapacity), size(0)
{
}

QuadTree::QuadTree(double width, int nodeCapacity) : QuadTree({0, 0, 1}, width, nodeCapacity)
{
}

size_t QuadTree::getSize() const
{
	return size;
}

bool QuadTree::add(const point_t &point)
{
	if (!inBounds(center, width, point))
	{
		return false;
	}

	if (points.size() < nodeCapacity && !hasChildren()) {
		points.push_back(point);
		size++;
		return true;
	}

	if (!hasChildren()) {
		subdivide();
	}

	for (const std::shared_ptr<QuadTree> &childPtr : children) {
		if (childPtr->add(point)) {
			size++;
			return true;
		}
	}

	// we should never get here
	return false;
}

bool QuadTree::remove(const point_t &point)
{
	if (!inBounds(center, width, point))
	{
		return false;
	}

	auto itr = std::find(points.begin(), points.end(), point);
	if (itr != points.end()) {
		points.erase(itr);
		size--;
		return true;
	}

	if (hasChildren()) {
		for (const std::shared_ptr<QuadTree> &childPtr : children) {
			if (childPtr->remove(point)) {
				size--;
				return true;
			}
		}
	}

	return false;
}

point_t QuadTree::getClosestWithin(const point_t &point, double areaSize)
{
	points_t pointsInRange = getPointsWithin(point, areaSize);
	if (pointsInRange.empty()) {
		return {0,0,0};
	} else {
		point_t closest = pointsInRange[0];
		double minDist = (closest - point).topRows<2>().norm();
		for (int i = 1; i < pointsInRange.size(); i++) {
			point_t p = pointsInRange[i];
			double dist = (p - point).norm();
			if (dist < minDist) {
				minDist = dist;
				closest = p;
			}
		}
		return closest;
	}
}

std::vector<point_t> QuadTree::getPointsWithin(const point_t &point, double areaSize)
{
	points_t ret;
	if (!inBounds(center, width, point))
	{
		return ret;
	}

	for (const point_t &p : points) {
		if (inBounds(point, areaSize, p)) {
			ret.push_back(p);
		}
	}

	if (hasChildren()) {
		for (const auto &child : children) {
			points_t pointsInChild = child->getPointsWithin(point, areaSize);
			ret.insert(ret.end(), pointsInChild.begin(), pointsInChild.end());
		}
	}

	return ret;
}

void QuadTree::subdivide()
{
	double d = width/4;
	for (int i = 0; i < 4; i++) {
		double dx = (i & 1) == 0 ? -1 : 1;
		double dy = (i & 0b10) == 0 ? -1 : 1;
		point_t newCenter = center + d * point_t(dx,dy,0);
		children[i] = std::make_shared<QuadTree>(newCenter, width/2, nodeCapacity);
	}
}

bool QuadTree::hasChildren()
{
	return static_cast<bool>(children[0]); // if one is initialized, all are
}
