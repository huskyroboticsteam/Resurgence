#include "Map.h"
#include <cmath>
#include<vector>
#include "Geometry.h"

void updateRobotLocation(node_ptr location)
{
    Map::robot_current_location = location;
}

void createNode(Point p)
{
    Map::map.push_back(Node{p, false});
}

double distanceBetweenNodes(Node a, Node b)
{
    return distance(a.location, b.location);
}

node_ptr Map::getNode(Point p)
{
    int i = 0;
    while(i < map.size())
    {
        if(this->map.at(i).location.equals(p)){
            return this->map.at(i);
        }
    }
    return nullptr;
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
    robot_current_location = *Map::getNode(Point{0,0});
}
