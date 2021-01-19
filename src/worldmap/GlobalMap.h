#pragma once

#include <vector>
#include <Eigen/Core>
#include "../simulator/utils.h"

class GlobalMap
{
public:
	GlobalMap();
	void addPoints(const transform_t &trf, const points_t &points);
	points_t getPoints() const;
private:
	points_t points;
	static points_t correctPointCloud(const points_t &pointCloud);
};
