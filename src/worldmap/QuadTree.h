#pragma once

#include <memory>

#include "../simulator/utils.h"

/**
 * Implements a quadtree for fast point lookups in 2D space. All points are expected
 * to be in {x,y,1} format.
 *
 * The quadtree represents a square area of a given size and supports roughly
 * logarithmic lookup times within that area.
 *
 * @see <a href="https://en.wikipedia.org/wiki/Quadtree">Quadtree</a>
 */
class QuadTree
{
public:
	/**
	 * Creates a new quadtree with the given area size centered at the origin.
	 * @param width The size (height and width) of the square area covered by this quadtree.
	 * @param nodeCapacity The number of points stored in each node.
	 * Too high means slower lookup times, too low means higher memory overhead.
	 */
	explicit QuadTree(double width, int nodeCapacity = 4);

	/**
	 * Creates a new quadtree with the given area size centered at the given point.
	 * @param center The center of the area spanned by this quadtree.
	 * @param width The size (height and width) of the square area covered by this quadtree.
	 * @param nodeCapacity The number of points stored in each node.
	 * Too high means slower lookup times, too low means higher memory overhead.
	 */
	QuadTree(point_t center, double width, int nodeCapacity = 16);

	/**
	 * Gets the total number of points stored in this quadtree.
	 * @return The number of points in this quadtree.
	 */
	size_t getSize() const;

	/**
	 * Gets if this quadtree is empty.
	 * @return True iff there are no points in this quadtree.
	 */
	bool empty() const;

	/**
	 * Gets a vector of all points in this quadtree. This is a linear operation, so
	 * don't use too often if there are a large number of points.
	 * Modifying this vector does not modify the tree.
	 * @return A vector consisting of all points in this quadtree.
	 */
	points_t getAllPoints() const;

	/**
	 * Add the given point to this quadtree.
	 * @param point The point to add.
	 * @return True if the point was successfully added. The only way this will return false
	 * is if the given point is outside of the area spanned by the quadtree.
	 */
	bool add(const point_t &point);

	/**
	 * Remove the first occurrence of the given point from this quadtree.
	 * @param point The point to remove.
	 * @return True if the point was successfully removed. False if the point was not in
	 * the quadtree to begin with.
	 */
	bool remove(const point_t &point);

	point_t getClosest(const point_t &point) const;

	/**
	 * Get the point in this quadtree that is closest to the given point.
	 * @param point The point for which to find the nearest neighbor.
	 * @param areaSize The size of the axis-aligned square bounding box centered at the given
	 * point in which to perform the search. Larger values will slow down the search.
	 * @return The nearest neighbor to the given point, or {0,0,0} if none found.
	 */
	point_t getClosestWithin(const point_t &point, double areaSize) const;

	/**
	 * Gets all points in the quadtree that lie in the axis-aligned square bounding box
	 * centered at the given point with the given size.
	 * @param point The center of the bounding box.
	 * @param areaSize The size of the bounding box.
	 * @return All points that lie within that bounding box. This vector may be empty.
	 */
	points_t getPointsWithin(const point_t &point, double areaSize) const;

private:
	// 0=SW,1=SE,2=NW,3=NE, so bit 1 is north-south and bit 0 is east-west
	// if one is initialized then all are initialized
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
