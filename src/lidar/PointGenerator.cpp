#include "PointGenerator.h"

#include <cmath>
#include <iostream>

namespace lidar
{
std::vector<PointXY> generateClusterRadius(float x0, float y0, float r, int num_pts)
{
	std::vector<PointXY> pts;
	for (int i = 0; i < num_pts; i++)
	{
		float x =
			x0 - r + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (2 * r)));
		float yAbsMax = sqrtf(powf(r, 2) - powf(x - x0, 2));
		float y = y0 - yAbsMax +
				  static_cast<float>(rand() / (static_cast<float>(RAND_MAX / (2 * yAbsMax))));
		PointXY p;
		p.x = x;
		p.y = y;
		pts.push_back(p);
	}
	return pts;
}

std::vector<PointXY> generateClusterLinear(float x0, float y0, float x1, float y1, float tol,
										   int num_pts)
{
	// create vector for line, which we will multiply by a random value to get a value
	float lin_x = x1 - x0;
	float lin_y = y1 - y0;

	std::vector<PointXY> pts;
	for (int i = 0; i < num_pts; i++)
	{
		float r = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
		float x = x0 + r * lin_x;
		float y = y0 + r * lin_y;
		pts.push_back(*generateClusterRadius(x, y, tol, 1).begin());
	}
	return pts;
}
} // namespace lidar
