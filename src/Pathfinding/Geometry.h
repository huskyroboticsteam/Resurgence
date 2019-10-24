#pragma once
#include <memory>

using node_ptr = std::shared_ptr<Node>;

struct Node
{
    Point location;//current location of node relative to origin
    bool is_blocked;//False is no obstacle, True is obstacle
    node_ptr north;
    node_ptr south;
    node_ptr east;
    node_ptr west;

    //Node(Point p, node_ptr north = nullptr, node_ptr south = nullptr, node_ptr east = nullptr, node_ptr west = nullptr, bool is_blocked = false);

    //when creating node, give null values to unknown parts
};

struct Point
{
    double x; //float?
    double y; //float?
};

double distance(Point a, Point b);