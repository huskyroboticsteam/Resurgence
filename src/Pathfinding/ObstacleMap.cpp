#include "ObstacleMap.h"
//vector and EnvMap.h included in PathMap.h


ObstacleMap::ObstacleMap(float rad, float step){
    this->radius = rad;
    this->step_size = step;
    for(int i = 0; i < (int)(radius * 2 / step + 1); i++)
    {
        obstacle_map.push_back(std::vector<bool>()); 
    }
    updateObstacleMap();
}


std::vector<std::shared_ptr<MapObstacle>> ObstacleMap::getData(float robotX, float robotY){
    //don't know final MapObstacle

    //following method needs global coords
    return findObjectsWithinSquare(half_width, robotX, robotY);//floats
    //std::vector<std::shared_ptr<MapObstacle>> findObjectsWithinRect(float x_lower_left, float y_lower_left, float x_upper_right, float y_upper_right) const;
}

void ObstacleMap::resetObstacleMap(){
    for (int i = 0; i < (int)(radius * 2 / step_size + 1); i++)
    {
        for (int j = 0; j < (int)(radius * 2 / step_size + 1); j++)
        {
           obstacle_map[i].push_back(false);
        }
    }
}

void ObstacleMap::updateObstacleMap(){
    float robotX, robotY;
    getRobotPosition(robotX, robotY);
    resetObstacleMap();
    MapObstacle obstacle;
    std::vector<std::shared_ptr<MapObstacle>> data = getData(robotX, robotY);
    for(std::shared_ptr<MapObstacle> obstacle_pointer : data)//don't know if can do
    {
        obstacle = *obstacle_pointer;
        //to do, following lines use double vector and mapObstacle objects
        // obstacle_map[(int)(robotY - obstacle.Y + radius/step_size)]
        // [(int)(robotX - obstacle.X + radius/step_size)] = true;
    }
}