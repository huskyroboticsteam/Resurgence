#include "PathMap.h"
//vector included in PathMap.h
static float const radius = 20; 


std::vector<std::shared_ptr<MapObstacle>> PathMap::getData(){
    float x, y;
    getRobotPosition(x, y);//assigns robot position to x and y
    //get robot location to input as coordinates

    //following method needs global coords
    std::vector<std::shared_ptr<MapObstacle>> temp = findObjectsWithinSquare(half_width, robotX, robotY);//floats
    //std::vector<std::shared_ptr<MapObstacle>> findObjectsWithinRect(float x_lower_left, float y_lower_left, float x_upper_right, float y_upper_right) const;
}

void PathMap::updatePathMap(){//is this necessary? only used to make pathing more readable

}

void PathMap::PathMap(){
    updatePathMap();
}
