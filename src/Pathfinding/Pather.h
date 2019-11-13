#pragma once
#include "Geometry.h"
#include <vector>
class Pather {
    public:
        // start searching from our current location up to the target.
        // We know our current location (Node type)
        // our neighbor location (the neighbor Node)
        // recursively checking on our neighbor Node until reach the target. 
        // find the distance between the nodes. 
        // move toward the closest neighbor, then
        // re-do the step above when get to that neighbor node. 

        Geometry::node_ptr getTheClosestLocation();
        double distanceBetweenNodes(Geometry::node_ptr currentLocation, Geometry::node_ptr target);
        bool isShorterDistance(double distanceA, double distanceB);
        bool isAObstacle(Geometry::node_ptr childeNode);
        Pather::Pather(std::vector<std::vector<bool>> map)
        // Looking for the neighbors, calculate the distance from current --> neighbors
        // assign the distance value to the interval value of each neighbor
        // mark that neighbor as visited.

    private:  
        std::vector<std::vector<bool>> map;
        struct point {
            int x;
            int y;
        }
        std::vector<point> visited
        
};
