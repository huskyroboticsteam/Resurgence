#include "ObstacleMap.h"
#include <cstdlib>

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

int ObstacleMap::transform(int val, bool direction)
{
    if(direction)
    {
        return val + (int)(step_size - (val % step_size));
    }else
    {
        return val - (int)(val % step_size);
    }
}

void ObstacleMap::update(std::vector<Point&> obstacles)
{
    int robotX = 0;
    int robotY = 0;
    //.getRobotPosition(robotX, robotY); // todo: figure out how to get robot position
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
    // x = (int)(point.x - robotX + radius/step_size);
    // y = (int)(point.y - robotY + radius/step_size);
    // // modifyObstacleMap() w/ valid points in range
}

inline void ObstacleMap::modifyObstacleMap(int x, int  y)
{
    //set four points surrounding given point as blocked
    //likely blocked areas will overlap from proximity of points in MapObstacle
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