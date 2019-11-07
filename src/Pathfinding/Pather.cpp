#include "Map.h"
#include "Pather.h"
#include <vector>

std::vector<std::vector<bool>> map;

Pather::Pather(std::vector<std::vector<bool>> map)
{
    this.map = map;
}

std::vector<Pather::point> Dijkstra() {
    std::vector<Pather::point> a;
    a.push_back(struct Point p = {map.size()/2, map.size()/2});
    // While a is not empty and we haven't found target:
        // Pull out first value from a and mark visited
        // Foreach neighbor, check if we have closer route
        // If so, update and add to a

    // When it's done, if we've found target:
        // Return the path we took to get to target!
}

double Pather::distanceToTarget() {
    
}

bool isShorterDistance(double distanceA, double distanceB) {

}

bool isAObstacle(node_ptr childNode) {

}