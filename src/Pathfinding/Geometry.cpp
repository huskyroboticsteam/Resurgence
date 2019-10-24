#include "Geometry.h"
#include <cmath>

double distance(Point a, Point b)
{
    return(sqrt((a.x - b.x)*(a.x - b.x) + (a.y - b.y) *  (a.y - b.y)));
}