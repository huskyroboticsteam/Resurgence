#include "Map.h"
#include "Pather.h"
#include <vector>

std::vector<std::vector<bool>> map;
std::vector<Pather::point> visited;
Pather::Pather(std::vector<std::vector<bool>> map)
{
    this.map = map;
}

std::vector<Pather::point> Dijkstra() {
    std::vector<Pather::point> pathToTarget;
    a.push_back(struct Point p = {map.size()/2, map.size()/2});
    // While a is not empty and we haven't found target:
        // Pull out first value from a and mark visited
        // Foreach neighbor, check if we have closer route
        // If so, update and add to a

    // When it's done, if we've found target:
        // Return the path we took to get to target!
    return pathToTarget; 
}

bool shorterDistance(Geometry::node_ptr nodePoint) {

}