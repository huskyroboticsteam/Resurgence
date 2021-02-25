#include "LinearQuadTree.h"

constexpr int hashLenBits = 6;
constexpr int hashDataBits = sizeof(unsigned long long) * 8 - hashLenBits;

LinearQuadTree::LinearQuadTree(double width) : width(width), hashes({})
{
}

void LinearQuadTree::add(const point_t &point)
{
	lochash_t hash = pointToHash(point);
	auto itr = std::lower_bound(hashes.begin(), hashes.end(), hash);
	hashes.insert(itr, hash);
}

void LinearQuadTree::remove(const point_t &point)
{
	lochash_t hash = pointToHash(point);
	auto itr = std::lower_bound(hashes.begin(), hashes.end(), hash);
	if (*itr == hash)
	{
		hashes.erase(itr);
	}
}

point_t LinearQuadTree::getClosest(const point_t &point)
{
	return point_t();
}

points_t LinearQuadTree::getPointsWithin(const point_t &point, double dist)
{
	return {};
}

point_t LinearQuadTree::hashToPoint(lochash_t loc) const
{
	point_t p = {0, 0, 1};
	unsigned long long hash = loc.getHash();
	constexpr int sizeBits = 8 * sizeof(lochash_t);
	constexpr unsigned long long mask = ~((1ull << (sizeBits-2))-1); // leftmost 2 bits are 1
	double width = this->width;
	const point_t dirs[] = {point_t(-1,-1,0), point_t(1,-1,0), point_t(-1,1,0), point_t(1,1,0)};
	for (int i = 0; i < hashDataBits; i++) {
		int idx = hash & mask;
		point_t dir = dirs[idx];
		p += dir * width/4.0;
		width /= 2.0;
		hash <<= 2;
	}
	p(2) = 1; // ensure the last index is 1.
	return p;
}

lochash_t LinearQuadTree::pointToHash(const point_t &point) const
{
	unsigned long long hash = hashDataBits/2; // automatically has full precision
	point_t center = {0,0,1};
	double width = this->width;
	const point_t dirs[] = {point_t(-1,-1,0), point_t(1,-1,0), point_t(-1,1,0), point_t(1,1,0)};
	for (int i = 0; i < hashDataBits/2; i++) {
		bool isTop = point(1) >= center(1);
		bool isRight = point(0) >= center(0);
		int idx = (isTop ? 0b10 : 0) | (isRight ? 1 : 0);
		hash <<= 2;
		hash |= idx;
		center += dirs[idx] * width/4.0;
		width /= 2.0;
	}
	return {hash};
}

lochash_t LinearQuadTree::getNeighbor(lochash_t locHash, Direction dir)
{
	unsigned long long loc = locHash.getHash();
	int len = locHash.getLen();
}

bool lochash_t::operator<(lochash_t other) const
{
	return getHash() < other.getHash();
}
bool lochash_t::operator>(lochash_t other) const
{
	return getHash() > other.getHash();
}
bool lochash_t::operator==(lochash_t other) const
{
	return hash == other.hash;
}

unsigned long long lochash_t::getHash() const
{
	return hash << hashLenBits;
}

uint8_t lochash_t::getLen() const
{
	return hash >> hashDataBits;
}
