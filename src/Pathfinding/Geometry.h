#pragma once

struct Node{
    Point coordinate;
    Node& north;
    Node& south;
    Node& east;
    Node& west;
    bool blocked;
};

struct Point{
    double x; //float?
    double y; //float?
};