#pragma once
//#include "Geometry.h"
#include <vector>
#include <queue>
#include <string>
//#include "ObstacleMap.h"
class Pather2 {
private:

public:
    struct point {
        int x;
        int y;
    };
            
    struct queueNode {
    //for use in BFS search
        point pt;
        int dist;
        std::queue<point> path;
    };
    point getPath(bool map[][21], point dest);
    point relocateDestination(point dest, int shrink_constant);
    int returnHeading(bool map[][21]);
};