#include "QuadTree.h"

QuadTree::QuadTree(double width, int precision) : precision(precision), root(point_t(), width, precision)
{
}

void QuadTree::add(const point_t &point)
{
	root.add(point);
}

void QuadTree::remove(const point_t &point)
{
	root.remove(point);
}

point_t QuadTree::getClosest(const point_t &point)
{
	return root.getClosest(point);
}

std::vector<point_t> QuadTree::getPointsWithin(const point_t &point, double dist)
{
	return root.getPointsWithin(point, dist);
}
QuadTree::Node::Node(const point_t &center, double width, int levelsToLeaf)
	: center(center), points(), width(width), levelsToLeaf(levelsToLeaf)
{
}

//int getChildIndex(bool isTop, bool isRight) {
//	return ((isTop ? 1 : 0) << 1) | (isRight ? 1 : 0);
//}
//
//std::shared_ptr<Node> & getChildPtr(const Node &node, const point_t &point) {
//	bool isRight = point(0) >= center(0);
//	bool isTop = point(1) >= center(1);
//	int index = ((isTop ? 1 : 0) << 1) | (isRight ? 1 : 0);
//	return children[index];
//}

void QuadTree::Node::add(const point_t &point)
{
	if (levelsToLeaf == 0) {
		points.insert(point);
	} else {
		double d = width/4;
		bool isRight = point(0) >= center(0);
		bool isTop = point(1) >= center(1);
		point_t newCenter = center + point_t(isRight ? d : -d, isTop ? d : -d, 1);
		std::shared_ptr<Node> &ptr = getChildPtr(point);
		if (!ptr) {
			ptr.reset(new Node(newCenter, width/2, levelsToLeaf-1));
		}
		ptr->add(point);
	}
}
void QuadTree::Node::remove(const point_t &point)
{
	if (levelsToLeaf == 0) {
		points.erase(point);
	} else {
		std::shared_ptr<Node> &ptr = getChildPtr(point);
		if (ptr) {
			ptr->remove(point);
		}
	}
}
std::shared_ptr<QuadTree::Node> & QuadTree::Node::getChildPtr(const point_t &point)
{
	bool isRight = point(0) >= center(0);
	bool isTop = point(1) >= center(1);
	int index = ((isTop ? 1 : 0) << 1) | (isRight ? 1 : 0);
	return children[index];
}
point_t QuadTree::Node::getClosest(const point_t &point)
{
	if (levelsToLeaf == 0) {
		if (points.empty()) {
			return {0,0,0};
		} else {
			point_t closest = {0,0,0};
			double minDist = 1e9;
			for (const point_t &p : points) {
				double dist = (p - point).norm();
				if (dist < minDist) {
					closest = p;
					minDist = dist;
				}
			}
			return closest;
		}
	} else {
		std::shared_ptr<QuadTree::Node> &ptr = getChildPtr(point);
		if (ptr) {
			point_t closest = ptr->getClosest(point);
			double minDist = (point - closest).norm();
			if (levelsToLeaf == 1) {
				for (const auto &childPtr : children) {
					if (childPtr) {
						point_t p = childPtr->getClosest(point);
						double dist = (p - point).norm();
						if (dist < minDist) {
							minDist = dist;
							closest = p;
						}
					}
				}
			}
			return closest;
		} else {
			return {0,0,0};
		}
	}
}
points_t QuadTree::Node::getPointsWithin(const point_t &point, double dist)
{
	return points_t();
}