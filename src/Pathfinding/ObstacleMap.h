#pragma once
#include <vector>
#include <math.h>
#include <iostream>
// #include <memory> // remove once EnvMap.h is included
// #include "MapObstacle.h"
#include "../mapping/EnvMap.h"

// constructed with reference or pointer to global EnvMap
// recieve list of obstacles within specified range around robot
// plot obstacles onto 2d grid
// pass grid onto pathing
// recreate grid as orientation/location changes

//increase bounding box 
class ObstacleMap{
    EnvMap& slam_map;
    constexpr static float radius = 10.0f;
    constexpr static float step_size = 1.0f;
    constexpr static int size = 21;
    bool obstacle_map[size][size];

private:
    // clears all obstacles from the map
    void resetObstacleMap();
    // retrieves obstacle location data
    std::vector<std::shared_ptr<const MapObstacle>> getData(float robotX, float robotY);
    // converts obstacle coordinates to indices
    // direction indicates +/- for rounding up or down to the nearest index, true is plus, false is -
    int transform(int val, bool direction);
    // sets the four elements around the given coordinates as blocked
    void modifyObstacleMap(int x, int y);

public:
    // clears the obstacle map and plots a new set of obstacles around the robot's current location
    void updateObstacleMap();
    // constructs an ObstacleMap with a reference to the global EnvMap
    ObstacleMap(EnvMap& envmap);
    // prints the map as a grid of 0/1, 0 = empty, 1 = blocked
    void print();
};