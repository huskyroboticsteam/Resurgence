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

node_ptr getNode(Point p)
{
    int i = 0;
    while(i < Map::map.size)
    {
        if(Map::map.get(i).location.equals(p)){
            return *Map::map.get(i);
        }
    }
    return nullptr;
}

Map()
{
    for(int i = -1 * Map::radius; i <= Map::radius; i += Map::step)
    {
        for(int j = -1 * Map::radius; j <= Map::radius; j += Map::step)
        {
            createNode(Point{i, j});
        }
    }
    robot_current_location = *Map.get(Point{0,0});
}
