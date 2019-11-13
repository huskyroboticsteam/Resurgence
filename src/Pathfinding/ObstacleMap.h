#pragma once
#include <vector>
#include <math.h>
#include <memory> // remove once EnvMap.h is included
//#include "EnvMap.h"

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
    float radius;
    float step_size;
    std::vector< std::vector<bool> > obstacle_map;

private:
    void resetObstacleMap();
    std::vector<std::shared_ptr<MapObstacle>> getData(float robotX, float robotY);
    int transform(int val, bool direction);

public:
    void updateObstacleMap();
    ObstacleMap(float rad, float step_size);
};

