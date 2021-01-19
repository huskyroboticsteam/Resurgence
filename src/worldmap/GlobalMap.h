#pragma once

#include <vector>
#include "../simulator/utils.h"

class GlobalMap
{
public:
	GlobalMap();
	void addPoints(const transform_t &trf, const points_t &points);
	const points_t &getPoints() const;
	point_t getClosest(const point_t &point) const;
private:
	points_t points;
	points_t correctPointCloud(const points_t &pointCloud);
};
