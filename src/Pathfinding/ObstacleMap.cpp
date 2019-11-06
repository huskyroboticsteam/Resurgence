#include "ObstacleMap.h"


//vector and EnvMap.h included in PathMap.h

std::vector<std::shared_ptr<MapObstacle>> ObstacleMap::getData(float robotX, float robotY){
    //assigns robot position to x and y
    //get robot location to input as coordinates

    //following method needs global coords
    return findObjectsWithinSquare(half_width, robotX, robotY);//floats
    //std::vector<std::shared_ptr<MapObstacle>> findObjectsWithinRect(float x_lower_left, float y_lower_left, float x_upper_right, float y_upper_right) const;
}

void ObstacleMap::resetObstacleMap(){
    for (int i = 0; i < ObstacleMap::obstacle_map.size(); i++){
    {
        for (int j = 0; i < obstacle_map[i].size(); i++)
        {
            obstacle_map[i][j] = false;
        }
    }
}

void ObstacleMap::updateObstacleMap(){
    float robotX, robotY;
    getRobotPosition(robotX, robotY);
    reset();
    MapObstacle obstacle;
    std::vector<std::shared_ptr<MapObstacle>> data = getData(robotX, robotY);
    for(std::shared_ptr<MapObstacle> obstacle_pointer : data)
    {
        obstacle = *obstacle_pointer;
        obstacle_map[(int)(robotY - obstacle.Y)][(int)(robotX - obstacle.X)] = true;
    }
}

ObstacleMap::ObstacleMap(float rad){
    this->radius = rad;
    this->obstacle_map = bool[2 * rad + 1][2 * rad + 1];
    updatePathMap();
}