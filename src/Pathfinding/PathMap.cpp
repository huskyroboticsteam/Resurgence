#include "PathMap.h"


//vector and EnvMap.h included in PathMap.h

std::vector<std::shared_ptr<MapObstacle>> PathMap::getData(){
    float robotX, robotY;
    getRobotPosition(robotX, robotY);//assigns robot position to x and y
    //get robot location to input as coordinates

    //following method needs global coords
    std::vector<std::shared_ptr<MapObstacle>> temp = findObjectsWithinSquare(half_width, robotX, robotY);//floats
    //std::vector<std::shared_ptr<MapObstacle>> findObjectsWithinRect(float x_lower_left, float y_lower_left, float x_upper_right, float y_upper_right) const;
}

void PathMap::updatePathMap(){//is this necessary? only used to make pathing more readable

}

void PathMap::PathMap(float rad){
    this->radius = rad;
    updatePathMap();
}
