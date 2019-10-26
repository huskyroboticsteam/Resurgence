#pragma once

#include <vector>
#include <fstream>
#include <cmath>
#include <vector>
#include <string>
#include <memory>
#include <unistd.h>
#include "PY2020/src/mapping/SyntheticLidar.h"

struct Point;

typedef struct Point{
    double xCoord;
    double yCoord;
} Point;

class Object
{
    public:
    std::vector<std::shared_ptr<Point>> pts;
    Point center;
    

    private:
};



const double sizeOfRobot = 2;
const int numOfPoints = 5; 

double ObjCluster();

std::shared_ptr<Point> polarTo2D(double r, double theta);
double calcDiff(std::shared_ptr<Point> p1, std::shared_ptr<Point> p2);
void filterPoints(std::vector<std::shared_ptr<Point>> points);
std::vector<std::shared_ptr<Point>> convertFrame(std::vector<std::shared_ptr<Polar2D>> frame);




