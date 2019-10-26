#pragma once
#include <memory>


struct Node
{
    Point location;//current location of node relative to origin
    bool is_blocked;//False is no obstacle, True is obstacle
    std::shared_ptr<Node> north;
    std::shared_ptr<Node> south;
    std::shared_ptr<Node> east;
    std::shared_ptr<Node> west;

    //when creating node, give null alues to unknown parts
};
using node_ptr = std::shared_ptr<Node>;

struct Point
{
    double x; //float?
    double y; //float?

    bool equals(Point p);
};

double distance(Point a, Point b);