#include "Geometry.h"
#include <cmath>

struct Node{
    Node(Point p, Node& north = nullptr, Node& south = nullptr, Node& east = nullptr, Node& west = nullptr, bool is_blocked = false)
    {
        location = p;
        north = north;
        south = south;
        east = east;
        west = west;
        is_blocked = is_blocked;
    }
};

struct Point{
    Point(double x, double y){
        x = x;
        y = y;
    }
};

double distance(Point a, Point b)
{
    return(sqrt((a.x - b.x)*(a.x - b.x) + (a.y - b.y) *  (a.y - b.y)));
}