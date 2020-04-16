#pragma once
//#include "Geometry.h"
#include <vector>
#include <queue>
#include <string>
#include "Point.h"
//#include "ObstacleMap.h"
class Pather2 {
private:

public:       
    struct queueNode {
    //for use in BFS search
        Point pt;
        int dist;
        std::queue<Point> path;
    };
    Point getPath(bool map[][21], Point dest);
    Point relocateDestination(Point dest, int shrink_constant);
    int returnHeading(bool map[][21]);
};