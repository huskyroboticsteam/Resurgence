#pragma once
#include<vector>
#include "Geometry.h"

class Map
{
    public:
        int radius = 20;
        int step = 1;

        std::vector<Node> map;
        node_ptr robot_current_location;
        void updateRobotLocation(node_ptr location);
        void createNode(Point p);
        double distanceBetweenNodes(node_ptr a, node_ptr b);

        Map();
        node_ptr getNode(Point p);
        
};
