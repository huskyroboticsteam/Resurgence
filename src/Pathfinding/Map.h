#pragma once
#include<vector>
#include "Geometry.h"

class Map
{
    public:
        std::vector<Node> map;
        Node& robot_current_location;
        void updateRobotLocation(node_ptr location);
        void createNode(Point p);
        double distanceBetweenNodes(node_ptr a, node_ptr b);

};
