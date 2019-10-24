#pragma once
#include "Geometry.h"
class Pather {
    public:
        

        
        
    private:   
    Node getTheClosestLocation();
    double distanceToTarget(node_ptr currentLocation, node_ptr target);
    bool isShorterDistance(double distanceA, double distanceB);

};
