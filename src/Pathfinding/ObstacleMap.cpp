#include "ObstacleMap.h"
#include <cstdlib>


//|----------------------------|
//|                            |
//|will Point have float values|
//|                            |
//|----------------------------|

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

//rounds up/down based on direction being true/false
int ObstacleMap::transform(int val, bool direction)
{
    if(direction)
    {
        return val + (int)(step_size - (val % step_size));
    }
    return val - (int)(val % step_size);
}

//rebuilds ObstacleMap with new Obstacles
void ObstacleMap::update(std::vector<Point&> obstacles)
{
    int robotX = 0;
    int robotY = 0;
    //.getRobotPosition(robotX, robotY); // todo: figure out how to get robot position, use filter, ask Benton
    resetObstacleMap();
    //filter which points we want to plot
    int x, y;
    for (int i = 0; i < obstacles.size(); i++) {
        if (obstacles[i].x <= (robotX + 10) && obstacles[i].x >= (robotX - 10) && obstacles[i].y <= (robotX + 10) && obstacles[i].y >= (robotX - 10)) {
            x = (int)(obstacles[i].x - robotX + radius/step_size);
            y = (int)(obstacles[i].y - robotY + radius/step_size);
            modifyObstacleMap(x, y);
        }
    }
    // loop over all points -- Point(x, y)
}


//set four points surrounding given point as blocked
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