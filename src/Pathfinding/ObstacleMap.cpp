#include "ObstacleMap.h"
#include <iostream>

//sets all values in ObstacleMap to false
inline void ObstacleMap::resetObstacleMap()
{
    for (int i = 0; i < size; i++)
    {
        for (int j = 0; j < size; j++)
        {
           ObstacleMap::obstacle_map[i][j] = false;
        }
    }
}

//returns true if coordinate is within bounds of square ObstacleMap
inline bool inBounds(int coordinate)
{
    return coordinate < ObstacleMap::size && coordinate >= 0;
}

//rebuilds ObstacleMap with given Obstacles
void ObstacleMap::update(const std::vector<PointXY>& obstacles, float robotX, float robotY)
{
    this->robotX = robotX;
    this->robotY = robotY;
    resetObstacleMap();
    int x, y;
    std::cout << obstacles.size() << std::endl;
    // for(int i = 0; i < obstacles.size(); i++) {
    //     PointXY &p = obstacles.at(i);
        
    // }



    for (const PointXY &p : obstacles) {
       x = static_cast<int>(p.x - robotX) + radius;
        y = static_cast<int>(p.y - robotY) + radius;
        // std::cout << "x: " << x << "  y: " << y << std::endl;
        if (inBounds(x) && inBounds(y))
        {
            modifyObstacleMap(x,y);
        } 
    }
    std::cout << "place 5" << std::endl;
}

//rounds given coordinates up/down to obstacle_map indices,
//sets four elements around given coordinates as blocked
inline void ObstacleMap::modifyObstacleMap(int x, int y)
{
    if (y + 1 < size && x + 1 < size) {
        obstacle_map[y + 1][x + 1] = true;
    }
    if (y + 1 < size && x >= 0) {
        obstacle_map[y + 1][x] = true;
    }
    if (y >= 0 && x + 1 < size) {
        obstacle_map[y][x + 1] = true;
    }
    if (y >= 0 && x >= 0) {
        obstacle_map[y][x] = true;
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

// int main(){
//     ObstacleMap obs;
//     std::cout << typeid(obs.obstacle_map).name() << std::endl;
// }