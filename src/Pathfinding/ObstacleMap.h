#pragma once
#include <vector>
#include <math.h>
#include <iostream>
#include "Point.h" //remove once we include slam files

// recieve list of points
// plot obstacles onto 2d grid
// pass grid onto pathing
// recreate grid as orientation/location changes

//uses default constructor with no params

//todo whatever we get from GPS convert to meters

class ObstacleMap{
<<<<<<< HEAD

private:
    //sets all values in ObstacleMap to false
    void resetObstacleMap();
    // rounds up/down based on direction being true/false, true = up, false = down
    int transform(int val, bool direction);
    // rounds given coordinates up/down to obstacle_map indices,
    //sets four elements around given coordinates as blocked
=======
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
>>>>>>> develop
    void modifyObstacleMap(int x, int y);
    //for getting robot position
    void getRobotPosition(float &robotX, float &robotY);//needs to use gps

public:
<<<<<<< HEAD
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
    void update(std::vector<Point> obstacles);
    //for testing purposes only, prints a visual representation of ObstacleMap,
    //1 = obstacle, 0 = empty
=======
    // clears the obstacle map and plots a new set of obstacles around the robot's current location
    void updateObstacleMap();
    // constructs an ObstacleMap with a reference to the global EnvMap
    ObstacleMap(EnvMap& envmap);
    // prints the map as a grid of 0/1, 0 = empty, 1 = blocked
>>>>>>> develop
    void print();
};