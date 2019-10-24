#pragma once
#include<vector>
#include "Geometry.h"

class Map
{
    public:
        std::vector<Node> map;
        Node& robot_current_location;

    void updateRobotLocation(Node& location);
    void createNode(Point p);
    double distanceBetweenNodes(Node& a, Node b);

};
