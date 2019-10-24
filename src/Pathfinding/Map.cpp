#include"Map.h"

void updateRobotLocation(Node& location)
{
    Map::robot_current_location = location;
}

void createNode(Point p)
{
    Map::map.push_back(new Node(p,nullptr,nullptr,nullptr,nullptr,false));
}

double distanceBetweenNodes(Node& a, Node b);

