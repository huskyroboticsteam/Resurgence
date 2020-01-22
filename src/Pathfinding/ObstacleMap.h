#pragma once
#include <vector>
#include <math.h>
#include <iostream>

// todo: include Point struct located in slam files

// recieve list of points
// plot obstacles onto 2d grid
// pass grid onto pathing
// recreate grid as orientation/location changes

//uses default constructor with no params

class ObstacleMap{

private:
    //sets all values in ObstacleMap to false
    void resetObstacleMap();
    // rounds up/down based on direction being true/false, true = up, false = down
    int transform(int val, bool direction);
    // rounds given coordinates up/down to obstacle_map indices, sets four elements around given coordinates as blocked
    void modifyObstacleMap(int x, int y);

public:
    //given values are expected to be in meters, but otherwise are in: ObstacleMap units
    // size = 2 * radius + 1
    constexpr static int radius = 10;
    // length/width of 1 element in obstacle_map
    constexpr static int step_size = 1;
    // length/width of obstacle_map
    constexpr static int size = 21;

    // call update() before for accurate map
    bool obstacle_map[size][size];
    //rebuilds ObstacleMap with given Obstacles
    void update(std::vector<Point&> obstacles);
    //for testing purposes only, prints a visual representation of ObstacleMap, 1 = obstacle, 0 = empty
    void print();
};