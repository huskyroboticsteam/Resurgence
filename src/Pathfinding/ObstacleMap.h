#pragma once
#include <vector>
#include <math.h>
#include <iostream>
// todo: include Point struct located in slam files

// use asserts

//make a graph object for pathing
//use 2d array not vectors

//make reference or pointer to global SLAM map obj
// recieve list of obstacles within specified range
// plot obstacles onto 2d grid
// pass grid onto pathing
// recreate grid as orientation/location changes

//increase bounding box 
class ObstacleMap{
    constexpr static int radius = 10.0;
    constexpr static int step_size = 1.0;
    constexpr static int size = 21;//(int)(2 * radius + 1);
    bool obstacle_map[size][size];

private:
    void resetObstacleMap();
    int transform(int val, bool direction);
    void modifyObstacleMap(int x, int y);

public:
    void update(std::vector<Point&> obstacles);
    ObstacleMap(std::vector<Point&> obstacles);
    void print();
    bool[][]& getMap();
};