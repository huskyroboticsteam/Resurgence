#include "Map.h"
#include <cmath>
#include<vector>
#include "Geometry.h"
#include<iostream>

void Map::updateRobotLocation(Node newPoint)
{
    robot_current_location = newPoint;
}

void Map::createNode(Point p)
{
    map.push_back(Node{p, false});
}

double Map::distanceBetweenNodes(node_ptr a, node_ptr b)
{
    return distance(a->location, b->location);
}

Node Map::getNode(Point p)
{
    int i = 0;
    while(i < map.size())
    {
        if(map.at(i).location.equals(p)){
            return map.at(i);
        }
        i++;
    }
}

Map::Map()
{
    for(int i = -1 * Map::radius; i <= Map::radius; i += Map::step)
    {
        for(int j = -1 * Map::radius; j <= Map::radius; j += Map::step)
        {
            createNode(Point{(double)i, (double)j});
        }
    }
    robot_current_location = getNode(Point{0,0});
}

int main(){
    Map map;
    std::cout << "it finished?"<<std::endl;
}
