#pragma once

#include <vector>

namespace Mapping
{
    struct Point2D
    {
        float x;
        float y;
        std::vector<Point2D> neighbors;
    };

    float dist(struct Point2D *p1, struct Point2D *p2);
}
