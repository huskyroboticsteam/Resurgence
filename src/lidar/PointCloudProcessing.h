#pragma once

#include "SyntheticLidar.h"

#include <vector>
#include <set>

namespace lidar
{

typedef struct PointXY
{
    float x;
    float y;
} PointXY;

typedef struct BoundingBox
{
    float xmin;
    float ymin;
    float xmax;
    float ymax;
} BoundingBox;

bool approxEqual(PointXY p, PointXY q);
float distance(float x0, float y0, float x1, float y1);
void localToGlobal(PointXY &p, float x_loc, float y_loc, float heading);
std::vector<std::set<PointXY>> clusterPoints(std::vector<PointXY> &pts, float sep_threshold);
std::vector<std::set<PointXY>> filterPointXYs(std::vector<PointXY> points, float sep_threshold);
void filterGroundPoints(std::vector<Polar2D> &pts, float scan_height, float slope_tol_rad);
float orientation(PointXY p, PointXY q, PointXY r);
std::vector<PointXY> convexHull(std::set<PointXY> &cluster);

}; // namespace Lidar
