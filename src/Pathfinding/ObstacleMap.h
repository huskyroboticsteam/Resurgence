#pragma once
#include <vector>
#include <math.h>
#include <iostream>
#include <memory> // remove once EnvMap.h is included
#include "MapObstacle.h"
#include "EnvMap.h"

//use asserts

//make a graph object for pathing
//use 2d array not vectors

//make reference or pointer to global SLAM map obj
// recieve list of obstacles within specified range
// plot obstacles onto 2d grid
// pass grid onto pathing
// recreate grid as orientation/location changes

//increase bounding box 
class ObstacleMap{
    //EnvMap& slam_map;
    constexpr static float radius = 10.0f;
    constexpr static float step_size = 1.0f;
    constexpr static int size = 21;//(int)(2 * radius + 1);
    bool obstacle_map[size][size];
    //change to 2d arrays

private:
    void resetObstacleMap();
    std::vector<std::shared_ptr<MapObstacle>> getData(float robotX, float robotY);
    int transform(int val, bool direction);

public:
    void updateObstacleMap();
    ObstacleMap(Envmap& envmap);
    void print();
};