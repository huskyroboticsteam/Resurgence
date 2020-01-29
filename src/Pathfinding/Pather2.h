#pragma once
//#include "Geometry.h"
#include <vector>
#include <queue>
#include <string>
//#include "ObstacleMap.h"
class Pather2 {
private:
    //bool[ObstacleMap.size][ObstacleMap.size] map;

    //point getPath(int mat[][], point dest);

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
};