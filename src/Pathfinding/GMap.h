#pragma once
#include <vector>

class GMap
{
    //GMap();
private:
    std::vector<Point> globalObstacleList;
    reduceObstacles();

public:
    addSLAMData();//ask slam for data type to be input
    getObstaclesWithinSquare(int squareRad, int x, int y);
}

struct Point
{
    int x;
    int y;
}