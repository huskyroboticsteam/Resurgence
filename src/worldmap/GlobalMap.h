#pragma once

#include <vector>
#include "../simulator/utils.h"
#include "TrICP.h"

class GlobalMap
{
public:
	GlobalMap();
	void addPoints(const transform_t &robotTrf, const points_t &points, double overlap);
	points_t getPoints() const;
	point_t getClosest(const point_t &point) const;
private:
	points_t points;
	transform_t trf;
	TrICP icp;
};
