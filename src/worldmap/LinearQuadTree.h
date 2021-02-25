#pragma once

#include "../simulator/utils.h"

struct lochash_t {
	// first 6 bits give length of hash, rest are hash
	unsigned long long hash;

	bool operator <(lochash_t hash) const;
	bool operator >(lochash_t hash) const;
	bool operator ==(lochash_t hash) const;
	unsigned long long getHash() const;
	uint8_t getLen() const;
};

enum Direction {
	N, E, S, W
};

class LinearQuadTree
{
public:
	explicit LinearQuadTree(double width);

	void add(const point_t &point);
	void remove(const point_t &point);
	point_t getClosest(const point_t &point);
	points_t getPointsWithin(const point_t &point, double dist);
	lochash_t getNeighbor(lochash_t loc, Direction dir);
	point_t hashToPoint(lochash_t hash) const;
	lochash_t pointToHash(const point_t &point) const;
private:
	std::vector<lochash_t> hashes;
	double width;
};
