#pragma once
#include <vector>

struct Vec2
{
public:
    int x;
    int y;
};

struct MapObstacle
{
public:
    std::vector<Vec2> points;
};