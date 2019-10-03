#include "Map2D.h"

#include <cmath>

namespace Mapping 
{
    float dist(struct Point2D *p1, struct Point2D *p2)
    {
        return sqrtf(powf(p1->x - p2->x, 2) + powf(p1->y - p2->y, 2));
    }
}
