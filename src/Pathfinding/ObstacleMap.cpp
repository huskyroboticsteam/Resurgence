#include "ObstacleMap.h"
//vector and EnvMap.h included in PathMap.h


ObstacleMap::ObstacleMap(float rad, float step)
{
    // get access to global EnvMap object
    this->radius = rad;
    this->step_size = step;
    for(int i = 0; i < (int)(radius * 2 / step + 1); i++)
    {
        obstacle_map.push_back(std::vector<bool>()); 
    }
    updateObstacleMap();
}


std::vector<std::shared_ptr<MapObstacle>> ObstacleMap::getData(float robotX, float robotY){
    //following method needs global coords
    // return findObjectsWithinSquare(this->radius, robotX, robotY);//floats
    return std::vector<std::shared_ptr<MapObstacle>>(); // delete once EnvMap.h is included, use findObjectsWthinSquare
    //std::vector<std::shared_ptr<MapObstacle>> findObjectsWithinRect(float x_lower_left, float y_lower_left, float x_upper_right, float y_upper_right) const;
}

void ObstacleMap::resetObstacleMap()
{
    for (int i = 0; i < (int)(radius * 2 / step_size + 1); i++)
    {
        for (int j = 0; j < (int)(radius * 2 / step_size + 1); j++)
        {
           obstacle_map[i].push_back(false);
        }
    }
}

int ObstacleMap::transform(int val, bool direction)
{
    //direction indicates +/- true is plus, false is -
    if(direction)
    {
        return val + (int)(step_size -  fmodf(val, step_size)); //(step_size - val % step_size);//fmodf
    }else
    {
        return val - (int)fmodf(val, step_size); //val % step_size;//fmodf
    }
}

void ObstacleMap::updateObstacleMap()
{
    float robotX, robotY;
    getRobotPosition(robotX, robotY); // from EnvMap
    resetObstacleMap();
    MapObstacle obstacle;
    int x, y;
    std::vector<std::shared_ptr<MapObstacle>> data = getData(robotX, robotY);
    // for(auto &obstacle_pointer : data)
    for (int i = 0; i < static_cast<int>(data.size()); i++)
    {
        // for (Vec2 point : (*obstacle_pointer).points)
        for (Vec2 point : data[i].points)
        {
            x = (int)(robotX - point.x + radius/step_size);
            y = (int)(robotY - point.y + radius/step_size);
            //set four points surrounding given point as blocked
            //likely blocked areas will overlap from proxity of points in MapObstacle
            obstacle_map[transform(y, true)][transform(x, true)] = true;
            obstacle_map[transform(y, true)][transform(x, false)] = true;
            obstacle_map[transform(y, false)][transform(x, true)] = true;
            obstacle_map[transform(y, false)][transform(x, false)] = true;
        }
    }
}