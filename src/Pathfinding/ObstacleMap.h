#pragma once
#include <vector>
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
    static float radius;
    static float step_size;
    std::vector< std::vector<bool> > obstacle_map;

private:
    void resetObstacleMap();
    std::vector<std::shared_ptr<MapObstacle>> getData(float robotX, float robotY);
    static int transform(int val, bool direction);

public:
    void updateObstacleMap();
    ObstacleMap(float rad, float step_size);
};

