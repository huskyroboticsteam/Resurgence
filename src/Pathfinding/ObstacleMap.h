#pragma once
#include <vector>
#include <math.h>
#include <iostream>
#include "Point.h"

// recieve list of points
// plot obstacles onto 2d grid
// pass grid onto pathing
// recreate grid as orientation/location changes

//uses default constructor with no params

//todo whatever we get from GPS convert to meters

class ObstacleMap{

private:
    //sets all values in ObstacleMap to false
    void resetObstacleMap();
    // sets four elements around coordinates of point that have
    //been truncated to x and y
    void modifyObstacleMap(int x, int y);
    //for retrieving robot position
    void updateRobotPosition();//needs to use gps

public:
    //given values are expected to be in meters, but otherwise are in: ObstacleMap units
    // size = 2 * radius + 1
    constexpr static int radius = 10;
    // length/width of obstacle_map
    constexpr static int size = 1 + 2 * radius;
    
    //robot coordinates ing GPS
    float robotX;//initialized to 0.0f upon construction
    float robotY;//initialized to 0.0f upon construction

    // call update() before for accurate map
    bool obstacle_map[size][size];
    
    //constructor
    ObstacleMap();
    // ObstacleMap(std::vector<Point>& obstacles);
    //rebuilds ObstacleMap with given Obstacles
    void update(std::vector<Point>& obstacles);
    //for testing purposes only, prints a visual representation of ObstacleMap,
    //1 = obstacle, 0 = empty
    void print();
};