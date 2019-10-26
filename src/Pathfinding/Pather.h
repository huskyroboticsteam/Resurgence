#pragma once
#include "Geometry.h"
class Pather {
    public:
        node_ptr getTheClosestLocation();
        double distanceToTarget(node_ptr currentLocation, node_ptr target);
        bool isShorterDistance(double distanceA, double distanceB);
        bool isAObstacle(node_ptr childeNode);
    private:   
    
};
