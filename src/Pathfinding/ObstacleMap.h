#pragma once
#include <vector>
#include <math.h>
#include <iostream>
// #include <memory> // remove once EnvMap.h is included
// #include "MapObstacle.h"

//to do: use asserts

//make a graph object for pathing
//use 2d array not vectors

//make reference or pointer to global SLAM map obj
// recieve list of obstacles within specified range
// plot obstacles onto 2d grid
// pass grid onto pathing
// recreate grid as orientation/location changes

//increase bounding box 
class ObstacleMap{
    GMap slam_map;
    constexpr static int radius = 10.0;
    constexpr static int step_size = 1.0;
    constexpr static int size = 21;//(int)(2 * radius + 1);
    bool obstacle_map[size][size];

private:
    void resetObstacleMap();
    std::vector<Point&> getData(int robotX, int robotY); // change return type (check w/ Assaf)
    int transform(int val, bool direction);
    void modifyObstacleMap(int x, int y);

public:
    void updateObstacleMap();
    ObstacleMap(GMap gMap);
    void print();
    bool[][]& getMap();
};