#include "ObstacleMap.h"
//vector and EnvMap.h included in PathMap.h


ObstacleMap::ObstacleMap(float rad, float step)
{
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
    return findObjectsWithinSquare(half_width, robotX, robotY);//floats
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

static int ObstacleMap::transform(int val, bool direction)
{
    //direction indicates +/- true is plus, false is -
    if(direction)
    {
        return val + (step_size - val % step_size);//fmodf
    }else
    {
        return val - val % step_size;//fmodf
    }
}

void ObstacleMap::updateObstacleMap()
{
    float robotX, robotY;
    getRobotPosition(robotX, robotY);
    resetObstacleMap();
    MapObstacle obstacle;
    int x, y;
    std::vector<std::shared_ptr<MapObstacle>> data = getData(robotX, robotY);
    for(std::shared_ptr<MapObstacle> obstacle_pointer : data)
    {
        for (Vec2 point : (*obstacle_pointer).points)
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