#include "Map2D.h"

#include <cmath>
#include <limits>
#include <queue>
#include <algorithm>

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
    auto cmp = [](std::shared_ptr<Point2D> p1, std::shared_ptr<Point2D> p2) {
        return p1->cost > p2->cost;
    };
    std::priority_queue<std::shared_ptr<Point2D>, std::vector<std::shared_ptr<Point2D>>, 
        decltype(cmp)> vq(cmp);

    for (std::shared_ptr<Point2D> p: map->vertices)
    {
        p == start ? p->cost = 0 : p->cost = std::numeric_limits<float>::infinity();
        p->prev = nullptr;
        vq.push(p);
    }

    while (!vq.empty())
    {
        std::shared_ptr<Point2D> curr = vq.top();
        vq.pop();
        for (Edge e: curr->neighbors)
        {
            float dist = curr->cost + e.dist;
            if (dist < e.target->cost)
            {
                e.target->cost = dist;
                e.target->prev = curr;
            }
        }
    }

    std::vector<std::shared_ptr<Point2D>> path;
    std::shared_ptr<Point2D> retrace = target;
    bool found = false;
    while (!found)
    {
        if (retrace->prev != nullptr)
        {
            path.push_back(retrace);
            retrace = retrace->prev;
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
