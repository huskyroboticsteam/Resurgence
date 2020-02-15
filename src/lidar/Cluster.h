#pragma once

#include "URGLidar.h"

#include <set>
#include <vector>
#include <memory>

typedef struct PointXY{
    float x;
    float y;
} PointXY;

std::shared_ptr<PointXY> polarToCartesian(float r, float theta);
float distance(std::shared_ptr<PointXY> p1, std::shared_ptr<PointXY> p2);
std::vector<std::set<std::shared_ptr<PointXY>>> clusterPoints(
    std::vector<std::shared_ptr<PointXY>> points, float sep_threshold);
std::vector<std::shared_ptr<PointXY>> convertFrame(std::vector<std::shared_ptr<Polar2D>> frame);




