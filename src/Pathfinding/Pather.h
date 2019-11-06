#pragma once
#include "Geometry.h"
class Pather {
    public:
        // start searching from our current location up to the target.
        // We know our current location (Node type)
        // our neighbor location (the neighbor Node)
        // recursively checking on our neighbor Node until reach the target. 
        // find the distance between the nodes. 
        // move toward the closest neighbor, then
        // re-do the step above when get to that neighbor node. 

        node_ptr getTheClosestLocation();
        double distanceToTarget(node_ptr currentLocation, node_ptr target);
        bool isShorterDistance(double distanceA, double distanceB);
        bool isAObstacle(node_ptr childeNode);
    private:   
            
};
