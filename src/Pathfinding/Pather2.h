#pragma once
#include <vector>
#include <queue>
#include <string>
#include "../math/PointXY.h"
#include "ObstacleMap.h"
class Pather2 {
private:
    PointXY getPath(PointXY dest);

public:       
    struct queueNode {
    //for use in BFS search
        PointXY pt;
        int dist;
        std::queue<PointXY> path;
    };
    ObstacleMap obsMap;
    // map is always 21x21 2d array

    // returns full path
    std::queue<PointXY> BFS(PointXY dest);
    
    PointXY mainBFS(const std::vector<PointXY>& obstacles, float robotX, float robotY, PointXY dest);

    PointXY getPath(const std::vector<PointXY>& obstacles, float robotX, float robotY, PointXY dest);
    
    
    PointXY relocateDestination(PointXY dest, int shrink_constant);
    // int returnHeading();
};