#include "ObstacleMap.h"
#include <iostream>
#include <cstdlib>

//gets robot position
inline void ObstacleMap::getRobotPosition(float robotX, float robotY)
{
    robotX = 10.0f;
    robotY = 10.0f;
}


//sets all values in ObstacleMap to false
void ObstacleMap::resetObstacleMap()
{
    for (int i = 0; i < size; i++)
    {
        for (int j = 0; j < size; j++)
        {
           ObstacleMap::obstacle_map[i][j] = false;
        }
    }
}

// rounds up/down based on direction being true/false
// true = up, false = down
int ObstacleMap::transform(int val, bool direction)
{
    if(direction)
    {
        return val + (int)(step_size - (val % step_size));
    }
    return val - (int)(val % step_size);
}

//rebuilds ObstacleMap with given Obstacles
void ObstacleMap::update(std::vector<Point> obstacles)
{
    float robotX = -2.0f;
    float robotY = -2.0f;
    std::cout << robotX << " " << robotY << std::endl;
    getRobotPosition(robotX, robotY);
    //.getRobotPosition(robotX, robotY); // todo: figure out how to get robot position, use filter, ask Benton
    resetObstacleMap();
    int x, y;
    for (int i = 0; i < obstacles.size(); i++) {
        //filter which obstacles we want to plot
        if (obstacles[i].x <= (robotX + radius) && obstacles[i].x >= (robotX - radius) 
        && obstacles[i].y <= (robotX + radius) && obstacles[i].y >= (robotX - radius)) {
            x = (int)(obstacles[i].x - robotX + radius/step_size);
            y = (int)(obstacles[i].y - robotY + radius/step_size);
            modifyObstacleMap(x, y);
        }
    }
}

//rounds given coordinates up/down to obstacle_map indices,
//sets four elements around given coordinates as blocked
inline void ObstacleMap::modifyObstacleMap(int x, int  y)
{
    if (transform(y, true) < size && transform(x, true) < size) {
        obstacle_map[transform(y, true)][transform(x, true)] = true;
    }
    if (transform(y, true) < size && transform(x, false) < size) {
        obstacle_map[transform(y, true)][transform(x, false)] = true;
    }
    if (transform(y, false) < size && transform(x, true) < size) {
        obstacle_map[transform(y, false)][transform(x, true)] = true;
    }
    if (transform(y, false) < size && transform(x, false) < size) {
        obstacle_map[transform(y, false)][transform(x, false)] = true;
    }
}

//for testing purposes only, prints a visual representation of ObstacleMap
//where 1 is an obstacle and 0 is empty
void ObstacleMap::print()
{
    for (int i = 0; i < size; i++)
    {
        for(int j = 0; j < size; j++)
        {
            if(ObstacleMap::obstacle_map[i][j])
            {
                std::cout << "1 ";
            }else
            {
                std::cout << "0 ";
            }
        }
        std::cout << std::endl;
    } 
}