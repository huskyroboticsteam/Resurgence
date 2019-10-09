#pragma once

#include <vector>
#include <memory>

namespace Mapping
{
struct Edge;
struct Point2D;

typedef struct Edge
{
    std::shared_ptr<Point2D> target;
    float dist;
} Edge;

typedef struct Point2D
{
    int id;
    float x;
    float y;
    float cost;
    std::shared_ptr<Point2D> prev;
    std::vector<Edge> neighbors;
} Point2D;

typedef struct Map2D
{
    std::vector<std::shared_ptr<Point2D>> vertices;
} Map2D;

float Dist(std::shared_ptr<Point2D> p1, std::shared_ptr<Point2D> p2);
void Connect(std::shared_ptr<Point2D> p1, std::shared_ptr<Point2D> p2);
std::vector<std::shared_ptr<Point2D>> ComputePath(std::shared_ptr<Map2D> map,
    std::shared_ptr<Point2D> start, std::shared_ptr<Point2D> target);

}
