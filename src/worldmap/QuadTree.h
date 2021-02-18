#pragma once

#include <memory>
#include <unordered_set>

#include "../simulator/utils.h"

class QuadTree
{
public:
	QuadTree(double width, int precision);

	void add(const point_t &point);
	void remove(const point_t &point);
	point_t getClosest(const point_t &point);
	points_t getPointsWithin(const point_t &point, double dist);

private:
	struct PointHasher {
		std::size_t operator()(const point_t &point) const {
			auto hash = std::hash<double>();
			return hash(point(0)) ^ hash(point(1));
		}
	};

	struct Node {
		Node(const point_t &center, double width, int levelsToLeaf);

		void add(const point_t &point);
		void remove(const point_t &point);
		point_t getClosest(const point_t &point);
		points_t getPointsWithin(const point_t &point, double dist);
		std::shared_ptr<Node> & getChildPtr(const point_t &point);

		std::shared_ptr<Node> children[4];
		int levelsToLeaf;
		point_t center;
		double width;
		std::unordered_set<point_t, PointHasher> points;
	};

	int precision;
	Node root;
};
