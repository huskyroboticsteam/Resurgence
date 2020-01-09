#pragma once
#include <vector>
//include "ekfslam.h"

class GMap
{
    //GMap();
    // add fields for robot location + get/set
private:
    std::vector<Point&> globalObstacleList;

public:
    int addPoint(Point& point);//ask slam for data type to be input
    Point& getPointByID(int id);
    std::vector<Point&> getObstaclesWithinSquare(int squareRad, int x, int y);
    void getRobotPosition(int& x, int& y)
}

struct Point
{
    int x;
    int y;
    int id;
}