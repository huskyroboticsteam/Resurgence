#include "Map2D.h"

#include <map>
#include <cmath>
#include <limits>
#include <queue>
#include <algorithm>
#include <iostream>

namespace Mapping
{
// euclidean distance between p1 and p2
float Dist(std::shared_ptr<Point2D> p1, std::shared_ptr<Point2D> p2)
{
    return sqrtf(powf(p1->x - p2->x, 2) + powf(p1->y - p2->y, 2));
}

// adds p1 to p2's neighbors and p2 to p1's neighbors
void Connect(std::shared_ptr<Point2D> p1, std::shared_ptr<Point2D> p2)
{
    p1->neighbors.push_back(p2);
    p2->neighbors.push_back(p1);
}

// computes the shortest path between the start and target points and returns path as a vector
// if there is no path from start to target, returns an empty vector
std::vector<std::shared_ptr<Point2D>> ComputePath(std::shared_ptr<Map2D> map,
    std::shared_ptr<Point2D> start, std::shared_ptr<Point2D> target)
{
    std::vector<std::shared_ptr<Point2D>> shortest_path;
    std::map<std::shared_ptr<Point2D>, float> distances;
    std::map<std::shared_ptr<Point2D>, std::shared_ptr<Point2D>> previous;
    auto cmp = [start](std::shared_ptr<Point2D> p1, std::shared_ptr<Point2D> p2) {
        return Dist(p1, start) > Dist(p2, start);
    };
    std::priority_queue<std::shared_ptr<Point2D>, std::vector<std::shared_ptr<Point2D>>,
        decltype(cmp)> vq(cmp);

    distances.emplace(start, 0);
    for (std::shared_ptr<Point2D> p: map->vertices)
    {
        distances.insert(std::pair<std::shared_ptr<Point2D>, float>(p,
            std::numeric_limits<float>::infinity()));
        vq.push(p);
    }

    while (!vq.empty())
    {
        std::shared_ptr<Point2D> curr = vq.top();
        vq.pop();

        for (std::shared_ptr<Point2D> p: curr->neighbors)
        {
            float dist = distances.find(curr)->second + Dist(p, curr);
            if (dist < distances.at(p)) 
            {
                distances[p] = dist;
                previous.insert(std::pair<std::shared_ptr<Point2D>, std::shared_ptr<Point2D>>(p,
                    curr));
            }
        }
    }

    bool found = false;
    std::shared_ptr<Point2D> retrace = target;
    while (!found)
    {
        shortest_path.push_back(retrace);
        std::map<std::shared_ptr<Point2D>, std::shared_ptr<Point2D>>::iterator itr =
            previous.find(retrace);
        if (itr != previous.end())
        {
            retrace = itr->second;
            found = retrace == start;
        }
        else
        {
            return std::vector<std::shared_ptr<Point2D>>();
        }
    }

    std::reverse(shortest_path.begin(), shortest_path.end());
    return shortest_path;
}
} // namespace Mapping