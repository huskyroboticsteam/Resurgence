#include "QuadTree.h"

#include <algorithm>

bool inBounds(const point_t &center, double size, const point_t &point)
{
	point_t diff = (point - center).array().abs();
	double halfSize = size / 2;
	return diff(0) <= halfSize && diff(1) <= halfSize;
}

bool boundsIntersect(const point_t &center1, double size1, const point_t &center2, double size2) {
	point_t absDiff = (center1 - center2).array().abs();
	return (absDiff.x() * 2 < (size1 + size2)) && (absDiff.y() * 2 < (size1 + size2));
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

bool QuadTree::empty() const
{
	return size == 0;
}

points_t QuadTree::getAllPoints() const
{
	points_t allPoints;
	allPoints.reserve(size);
	getAllPoints(*this, allPoints);
	return allPoints;
}

void QuadTree::getAllPoints(const QuadTree &tree, points_t &allPoints) const
{
	allPoints.insert(allPoints.end(), tree.points.begin(), tree.points.end());
	if (tree.hasChildren())
	{
		for (const auto &ptr : tree.children)
		{
			getAllPoints(*ptr, allPoints);
		}
	}
}

bool QuadTree::add(const point_t &point)
{
	if (!inBounds(center, width, point))
	{
		return false;
	}

	if (points.size() < nodeCapacity && !hasChildren())
	{
		points.push_back(point);
		size++;
		return true;
	}

	if (!hasChildren())
	{
		subdivide();
	}

	for (const std::shared_ptr<QuadTree> &childPtr : children)
	{
		if (childPtr->add(point))
		{
			size++;
			return true;
		}
	}

	// this only happens if the given point is outside of the entire tree
	return false;
}

bool QuadTree::remove(const point_t &point)
{
	if (!inBounds(center, width, point))
	{
		return false;
	}

	auto itr = std::find(points.begin(), points.end(), point);
	if (itr != points.end())
	{
		points.erase(itr);
		size--;
		return true;
	}

	if (hasChildren())
	{
		for (const std::shared_ptr<QuadTree> &childPtr : children)
		{
			if (childPtr->remove(point))
			{
				size--;
				return true;
			}
		}
	}

	return false;
}

point_t QuadTree::getClosestWithin(const point_t &point, double areaSize) const
{
	// TODO: maybe make this iteratively increase search radius, to make it faster
	points_t pointsInRange = getPointsWithin(point, areaSize);
	if (pointsInRange.empty())
	{
		return {0, 0, 0};
	}
	else
	{
		point_t closest = pointsInRange[0];
		double minDist = (closest - point).topRows<2>().norm();
		for (int i = 1; i < pointsInRange.size(); i++)
		{
			point_t p = pointsInRange[i];
			double dist = (p - point).norm();
			if (dist < minDist)
			{
				minDist = dist;
				closest = p;
			}
		}
		return closest;
	}
}

std::vector<point_t> QuadTree::getPointsWithin(const point_t &point, double areaSize) const
{
	points_t ret;
	if (!boundsIntersect(center, width, point, areaSize))
	{
		return ret;
	}

	for (const point_t &p : points)
	{
		if (inBounds(point, areaSize, p))
		{
			ret.push_back(p);
		}
	}

	if (hasChildren())
	{
		for (const auto &child : children)
		{
			points_t pointsInChild = child->getPointsWithin(point, areaSize);
			ret.insert(ret.end(), pointsInChild.begin(), pointsInChild.end());
		}
	}

	return ret;
}

void QuadTree::subdivide()
{
	double d = width / 4;
	for (int i = 0; i < 4; i++)
	{
		double dx = (i & 1) == 0 ? -1 : 1;
		double dy = (i & 0b10) == 0 ? -1 : 1;
		point_t newCenter = center + d * point_t(dx, dy, 0);
		children[i] = std::make_shared<QuadTree>(newCenter, width / 2, nodeCapacity);
	}
}

bool QuadTree::hasChildren() const
{
	return static_cast<bool>(children[0]); // if one is initialized, all are
}
