#include "ObstacleMap.h"
#include <cstdlib>
#include <iostream>
//vector and EnvMap.h included in PathMap.h

inline std::vector<Point&> ObstacleMap::getData(int centerX, int centerY) // change return type (check w/ assaf)
                                                                                                    // get robot x/y from gMap?
{
    return gMap.getObstaclesWithinSquare(this->radius, centerX, centerY); 
}

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

void ObstacleMap::updateObstacleMap()
{
    int robotX = 0;
    int robotY = 0;
    gMap.getRobotPosition(robotX, robotY); // from gMap
    resetObstacleMap();
    std::vector<Point&> data = getData(robotX, robotY); // change type for data
    std::cout << data.size();
    int x, y;
    // edit loop once GMap data type is determined
    for (int i = 0; i < data.size(); i++)
    {
        for (int j = 0; j < data[i]->points.size(); j++)
        {
            Vec2 point = data[i]->points[j];
            x = (int)(point.x - robotX + radius/step_size);
            y = (int)(point.y - robotY + radius/step_size);
            modifyObstacleMap(x,y);   
        }
    }
}

inline void ObstacleMap::modifyObstacleMap(int x, int  y)
{
    //set four points surrounding given point as blocked
    //likely blocked areas will overlap from proxity of points in MapObstacle
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

ObstacleMap::ObstacleMap(GMap globalMap) : gMap(globalMap)
{
    updateObstacleMap();
}


// int main()
// {
//     ObstacleMap map = ObstacleMap();
//     map.print();
// };


//old tester mapobstacle
// #pragma once
// #include <vector>

// struct Vec2
// {
// public:
//     float x;
//     float y;
// };

// struct MapObstacle
// {
// public:
//     std::vector<Vec2> points;
// };