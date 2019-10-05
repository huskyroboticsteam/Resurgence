#include "Map2D.h"

#include <map>
#include <cmath>
#include <limits>
#include <queue>
#include <algorithm>
#include <iostream>

namespace Mapping
{
float Dist(std::shared_ptr<Point2D> p1, std::shared_ptr<Point2D> p2)
{
    return sqrtf(powf(p1->x - p2->x, 2) + powf(p1->y - p2->y, 2));
}

void Connect(std::shared_ptr<Point2D> p1, std::shared_ptr<Point2D> p2)
{
    float dist = Dist(p1, p2);
    Edge e1;
    e1.target = p2;
    e1.dist = dist;
    p1->neighbors.push_back(e1);
    Edge e2;
    e2.target = p1;
    e2.dist = dist;
    p2->neighbors.push_back(e2);
}

std::vector<std::shared_ptr<Point2D>> ComputePath(std::shared_ptr<Map2D> map,
    std::shared_ptr<Point2D> start, std::shared_ptr<Point2D> target)
{
    std::map<int, int> prev;

    std::vector<float> distances(map->vertices.size(), std::numeric_limits<float>::infinity());
    distances[start->id] = 0;

    auto cmp = [start](std::shared_ptr<Point2D> p1, std::shared_ptr<Point2D> p2) {
        return Dist(p1, start) > Dist(p2, start);
    };
    std::priority_queue<std::shared_ptr<Point2D>, std::vector<std::shared_ptr<Point2D>>, 
        decltype(cmp)> vq(cmp);
    for (std::shared_ptr<Point2D> p: map->vertices)
    {
        vq.push(p);
    }

    while (!vq.empty())
    {
        std::shared_ptr<Point2D> curr = vq.top();
        vq.pop();
        for (Edge e: curr->neighbors)
        {
            float dist = distances[curr->id] + e.dist;
            if (dist < distances[e.target->id])
            {
                distances[e.target->id] = dist;
                prev.insert(std::pair<int, int>(e.target->id, curr->id));
            }
        }
    }

    std::vector<std::shared_ptr<Point2D>> path;
    int retrace = target->id;
    bool found_target = false;
    while (!found_target)
    {
        path.push_back(map->vertices[retrace]);
        std::map<int, int>::iterator itr = prev.find(retrace);
        if (itr != prev.end())
        {
            retrace = itr->second;
            found_target = retrace == start->id;
        }
        else
        {
            return std::vector<std::shared_ptr<Point2D>>();
        }
    }

    std::reverse(path.begin(), path.end());
    return path;
}
} // namespace Mapping
