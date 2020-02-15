#pragma once

<<<<<<< HEAD
=======
#include "URGLidar.h"

>>>>>>> 9394cc3bf17070e0604cf744a9bcbf314ba87420
#include <vector>
#include <set>

namespace lidar
{

typedef struct PointXY
{
    float x;
    float y;
} PointXY;

typedef struct Polar2D
{
    double r, theta;
} Polar2D;

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
std::vector<std::vector<PointXY>> clusterPoints(std::vector<PointXY> &pts, float sep_threshold);
std::vector<std::vector<PointXY>> filterPointXYs(std::vector<PointXY> points, float pitch_rad,
                                                 float sep_threshold);
void filterGroundPoints(std::vector<Polar2D> &pts, float scan_height, float slope_tol_rad);
std::vector<PointXY> convexHull(std::set<PointXY> &cluster);

}; // namespace lidar
