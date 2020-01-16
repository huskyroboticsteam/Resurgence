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
    //given values are expected to be in meters, but otherwise are in: ObstacleMap units
    constexpr static int radius = 10;
    constexpr static int step_size = 1;
    constexpr static int size = 21;//(int)(2 * radius + 1);

    bool obstacle_map[size][size];


private:
    void resetObstacleMap();
    int transform(int val, bool direction);
    void modifyObstacleMap(int x, int y);

public:
    void update(std::vector<Point&> obstacles);
    void print();
};