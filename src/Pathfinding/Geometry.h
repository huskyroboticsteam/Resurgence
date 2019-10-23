#pragma once

struct Node{
    Point location;//current location of node relative to origin
    Node& north;
    Node& south;
    Node& east;
    Node& west;
    bool isBlocked;//False is no obstacle, True is obstacle

    //when creating node, give null values to unknown parts
};

struct Point{
    double x; //float?
    double y; //float?
};

double distance(Point a, Point b);