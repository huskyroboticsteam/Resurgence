#pragma once
#include<vector>
#include "Geometry.h"
using node_ptr = std::shared_ptr<Node>;

class Map
{
    public:
        int radius = 20;
        int step = 1;
        std::vector<Node> map;
        Node robot_current_location;
        void updateRobotLocation(Node newPoint);
        void createNode(Point p);
        double distanceBetweenNodes(node_ptr a, node_ptr b);
<<<<<<< HEAD
        void dijkstra();
=======

        Node getNode(Point p);
        Map();
>>>>>>> 9f910c3ddb4bc9c8bf82e1d9f6b92f2958c71dbb
};
