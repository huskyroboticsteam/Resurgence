#include "QuadTree.h"

#include <algorithm>
#include <loguru.hpp>
#include <stack>

using namespace navtypes;

// checks if the given point is contained in the specified square axis-aligned bounding box
bool inBounds(const point_t& center, double size, const point_t& point) {
	point_t diff = (point - center).array().abs();
	double halfSize = size / 2;
	return diff(0) <= halfSize && diff(1) <= halfSize;
}

// checks if two square axis-aligned bounding boxes intersect
bool boundsIntersect(const point_t& center1, double size1, const point_t& center2,
					 double size2) {
	point_t absDiff = (center1 - center2).array().abs();
	return (absDiff.x() * 2 <= (size1 + size2)) && (absDiff.y() * 2 <= (size1 + size2));
}

QuadTree::QuadTree(point_t center, double width, int nodeCapacity)
	: center(std::move(center)), width(width), points({}), nodeCapacity(nodeCapacity),
	  size(0) {}

QuadTree::QuadTree(double width, int nodeCapacity)
	: QuadTree({0, 0, 1}, width, nodeCapacity) {}

size_t QuadTree::getSize() const {
	return size;
}

bool QuadTree::empty() const {
	return size == 0;
}

points_t QuadTree::getAllPoints() const {
	points_t allPoints;
	allPoints.reserve(size);
	getAllPoints(*this, allPoints);
	return allPoints;
}

void QuadTree::getAllPoints(const QuadTree& tree, points_t& allPoints) const {
	allPoints.insert(allPoints.end(), tree.points.begin(), tree.points.end());
	if (tree.hasChildren()) {
		for (const auto& ptr : tree.children) {
			getAllPoints(*ptr, allPoints);
		}
	}
}

point_t QuadTree::getArbitraryPoint() const {
	point_t zero = point_t(0, 0, 0);
	if (!points.empty()) {
		return points[0];
	} else if (hasChildren()) {
		for (const auto& child : children) {
			point_t p = child->getArbitraryPoint();
			if (p != zero) {
				return p;
			}
		}
	}

	return zero;
}

bool QuadTree::add(const point_t& point) {
	if (!inBounds(center, width, point)) {
		return false;
	}

	// if we have room for this point in this node, add it
	if (points.size() < nodeCapacity && !hasChildren()) {
		points.push_back(point);
		size++;
		return true;
	}

	// create children if this node doesn't have any
	if (!hasChildren()) {
		subdivide();
	}

	// add this node to the child that contains it
	for (const std::shared_ptr<QuadTree>& childPtr : children) {
		if (childPtr->add(point)) {
			size++;
			return true;
		}
	}

	// this should never happen
	return false;
}

bool QuadTree::remove(const point_t& point) {
	if (!inBounds(center, width, point)) {
		return false;
	}

	// if the point is in this node, erase it and return
	auto itr = std::find(points.begin(), points.end(), point);
	if (itr != points.end()) {
		points.erase(itr);
		size--;
		return true;
	}

	// search through children for this point
	if (hasChildren()) {
		for (const std::shared_ptr<QuadTree>& childPtr : children) {
			if (childPtr->remove(point)) {
				size--;
				return true;
			}
		}
	}

	// the point is not stored in this tree
	return false;
}

// gets the index of the child node of the node centered at `center` that contains `point`
int getChildIdx(const point_t& center, const point_t& point) {
	// 0=SW,1=SE,2=NW,3=NE, so bit 1 is north-south and bit 0 is east-west
	bool isTop = point.y() >= center.y();
	bool isRight = point.x() >= center.x();
	return (isTop ? 0b10 : 0) | (isRight ? 1 : 0);
}

point_t QuadTree::getClosest(const point_t& point) const {
	// if there are no children just search through this tree's points
	if (!hasChildren()) {
		point_t closest = {0, 0, 0};
		double minDist = std::numeric_limits<double>::infinity();
		for (const point_t& p : points) {
			double dist = (p - point).topRows<2>().norm();
			if (dist < minDist) {
				closest = p;
				minDist = dist;
			}
		}
		return closest;
	}
	const QuadTree* tree = this;
	std::stack<const QuadTree*> stack;
	stack.push(tree);

	// traverse down the tree until we find a square w/o children containing the search point
	while (tree->hasChildren()) {
		QuadTree* child = tree->children[getChildIdx(tree->center, point)].get();
		stack.push(child);
		tree = child;
	}

	// traverse back up until we find a nonempty square (usually the first one)
	while (!stack.empty() && tree->empty()) {
		tree = stack.top();
		stack.pop();
	}

	// if there are no points in this tree, return {0,0,0}
	if (tree->empty()) {
		CHECK_EQ_F(size, 0); // this should only happen if the tree is empty
		return {0, 0, 0};
	} else {
		CHECK_F(!tree->empty()); // should be guaranteed
		// choose an arbitrary point
		point_t p = tree->getArbitraryPoint();
		// Search area is square, so using the distance guarantees that the closest is found
		// multiply by two because this is half side-length
		double areaSize = 2 * (p - point).topRows<2>().norm();
		// get the closest of all points within this area
		return getClosestWithin(point, areaSize);
	}
}

point_t QuadTree::getClosestWithin(const point_t& point, double areaSize) const {
	points_t pointsInRange = getPointsWithin(point, areaSize);
	if (pointsInRange.empty()) {
		// no points in range
		return {0, 0, 0};
	} else {
		// compute the nearest neighbor in linear time (on a much smaller subset of points)
		point_t closest = pointsInRange[0];
		double minDist = (closest - point).topRows<2>().norm();
		for (size_t i = 1; i < pointsInRange.size(); i++) {
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

std::vector<point_t> QuadTree::getPointsWithin(const point_t& point, double areaSize) const {
	points_t ret;
	// if the given bounding box doesn't intersect this node's area, return empty vector
	if (!boundsIntersect(center, width, point, areaSize)) {
		return ret;
	}

	// add any points from this node in the search area to the vector
	for (const point_t& p : points) {
		if (inBounds(point, areaSize, p)) {
			ret.push_back(p);
		}
	}

	// search through children, if they exist
	if (hasChildren()) {
		for (const auto& child : children) {
			points_t pointsInChild = child->getPointsWithin(point, areaSize);
			ret.insert(ret.end(), pointsInChild.begin(), pointsInChild.end());
		}
	}

	return ret;
}

bool QuadTree::hasPointWithin(const point_t& point, double dist) const {
	// if the given bounding box doesn't intersect this node's area, return false
	if (empty() || !boundsIntersect(center, width, point, dist * 2)) {
		return false;
	}

	// check any points from this node
	for (const point_t& p : points) {
		if ((p - point).topRows<2>().norm() <= dist)
			return true;
	}

	// search through children, if they exist
	if (hasChildren()) {
		for (const auto& child : children) {
			if (child->hasPointWithin(point, dist))
				return true;
		}
	}

	return false;
}

void QuadTree::subdivide() {
	double d = width / 4;
	for (int i = 0; i < 4; i++) {
		// 0=SW,1=SE,2=NW,3=NE, so bit 1 is north-south and bit 0 is east-west
		double dx = (i & 1) == 0 ? -1 : 1;
		double dy = (i & 0b10) == 0 ? -1 : 1;
		point_t newCenter = center + d * point_t(dx, dy, 0);
		children[i] = std::make_shared<QuadTree>(newCenter, width / 2, nodeCapacity);
	}
}

bool QuadTree::hasChildren() const {
	return static_cast<bool>(children[0]); // if one child is initialized, all are
}
