#pragma once

#include "../math/PointXY.h"
#include "../navtypes.h"

#include <vector>

class Polar2D;

namespace lidar {

typedef struct BoundingBox {
	float xmin;
	float ymin;
	float xmax;
	float ymax;
} BoundingBox;

bool approxEqual(PointXY p, PointXY q);
float distance(float x0, float y0, float x1, float y1);
PointXY polarToCartesian(Polar2D p);
navtypes::point_t polarToCartesian2(Polar2D p);
void localToGlobal(PointXY& p, float x_loc, float y_loc, float heading);
std::vector<std::vector<PointXY>> clusterPoints(std::vector<PointXY>& pts,
												float sep_threshold);
std::vector<std::vector<PointXY>> clusterOrderedPoints(std::vector<PointXY>& points,
													   float sep_threshold);
void filterGroundPoints(std::vector<Polar2D>& pts, float scan_height, float slope_tol_rad);
std::vector<PointXY> convexHull(std::vector<PointXY>& cluster);

}; // namespace lidar
