#pragma once
//#include "Geometry.h"
#include <vector>
#include <queue>
#include <string>
#include "../math/PointXY.h"
//#include "ObstacleMap.h"
class Pather2 {
private:

public:       
    struct queueNode {
    //for use in BFS search
        PointXY pt;
        int dist;
        std::queue<PointXY> path;
    };
    // returns full path
    std::queue<PointXY> BFS(bool map[][21], PointXY dest);
    PointXY mainBFS(bool map[][21], PointXY dest);
    PointXY getPath(bool map[][21], PointXY dest);
    PointXY relocateDestination(PointXY dest, int shrink_constant);
    int returnHeading(bool map[][21]);
};
