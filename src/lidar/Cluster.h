#pragma once

#include "lidar/SyntheticLidar.h"

#include <vector>
#include <set>
#include <fstream>
#include <cmath>
#include <vector>
#include <string>
#include <memory>
#include <unistd.h>

struct Point;

typedef struct Point{
    float xCoord;
    float yCoord;
} Point;

class Object
{
    public:
        std::vector<std::shared_ptr<Point>> pts;
        Point center;
    

    private:
};



const float sizeOfRobot = 2;
const int numOfPoints = 5; 

float ObjCluster();

std::shared_ptr<Point> polarTo2D(float r, float theta);
float distance(std::shared_ptr<Point> p1, std::shared_ptr<Point> p2);
void filterPoints(std::vector<std::shared_ptr<Point>> points);
std::vector<std::shared_ptr<Point>> convertFrame(std::vector<std::shared_ptr<Polar2D>> frame);




