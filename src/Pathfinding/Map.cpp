#include "Map.h"
#include <cmath>

void updateRobotLocation(Node& location)
{
    Map::robot_current_location = location;
}

void createNode(Point p)
{
    Map::map.push_back(Node{p, false});
}

double distanceBetweenNodes(Node& a, Node b)
{
    return
}

