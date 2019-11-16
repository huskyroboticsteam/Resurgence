#pragma once
#include <vector>

struct Vec2
{
public:
    float x;
    float y;
};

struct MapObstacle
{
public:
    std::vector<Vec2> points;
};