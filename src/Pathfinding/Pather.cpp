#include "Map.h"
#include "Pather.h"
#include <vector>


Pather::Pather(std::vector<std::vector<bool>> map)
{
    this.map = map;
}

std::vector<Pather::point> Dijkstra(int map[5][5]) {
    std::vector<Pather::point> visited;
    priorityqueue <Pather::queueElement> active;
    struct Pather::point src = {map.size()/2, map.size()/2};
    //need endpoint
    struct Pather::queueElement starts = {NULL, src, 0};
    active.emplace(starts);
    while (!active.empty() /*&& it is not the target*/){
        Pather::queueElement min = active.top(); //get minimum distance 
        double distance = min.distance;
        std::vector<Pather::point> pointList = getNeighbors(min.activeNode);
        //go through the point list 
        //check if it is visied --> if yes, ignore.
        //Otherwise, get distance = min.distance + constant;
        //Then create elemeneQueue for it --> add to active list.
    }
    // While a is not empty and we haven't found target:
        // Pull out first value from a and mark visited
        // Foreach neighbor, check if we have closer route
        // If so, update and add to a

    // When it's done, if we've found target:
        // Return the path we took to get to target!
    return pathToTarget; 
}

std::vector<Pather::point> getNeighbors(Pather::point a) {
    if (a.x + 1 < map.size() && map[a.x + 1][a.y] == true) {
        list.add(struct Pather::point{a.x + 1, a.y});
    }

    if (a.y + 1 < map.size() && map[a.x][a.y + 1] == true) {
        list.add(struct Pather::point{a.x, a.y + 1});
    }

    if (a.x - 1 >= 0 && map[a.x - 1][a.y] == true) {
        list.add(struct Pather::point{a.x - 1, a.y});
    }

    if (a.y - 1 >= 0 && map[a.x][a.y - 1] == true) {
        list.add(struct Pather::point{a.x, a.y - 1});
    }

}

//get the location of 2 locations on the map, and put them into elementPriority object
//and return it. 
Pather::queueElement processData(Pather::point locationA, Pather::point locationB) {
    double distance = calculateDistance(locationA, locationB);
    return Pather::queueElement elementP = {locationA, locationB, distance};
}

//calculate and return the distance between 2 location A and B
double Pather::calculateDistance(Pather::point locationA, Pather::point locationB) {
    //do some Math over to calculate and return the distance
    double distance = 0.0; 
    return distance;
}