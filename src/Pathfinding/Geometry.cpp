#include "Geometry.h"
#include <cmath>



double distance(Point a, Point b)
{
    return(sqrt((a.x - b.x)*(a.x - b.x) + (a.y - b.y) *  (a.y - b.y)));
}

bool Point::equals(Point p)
{
    return (fabs(p.x - this->x) < err && fabs(p.y - this->y) < err){
}