#pragma once

struct Node
{
    Point location;//current location of node relative to origin
    Node& north;
    Node& south;
    Node& east;
    Node& west;
    bool is_blocked;//False is no obstacle, True is obstacle

    Node(Point p, Node& north = nullptr, Node& south = nullptr, Node& east = nullptr, Node& west = nullptr, bool is_blocked = false);

    //when creating node, give null values to unknown parts
};

struct Point
{
    double x; //float?
    double y; //float?

    Point(double x, double y);
};

double distance(Point a, Point b);