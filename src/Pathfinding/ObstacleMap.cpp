#include "ObstacleMap.h"
#include <cstdlib>
//vector and EnvMap.h included in PathMap.h


//debug here, gives nullptrs
std::vector<std::shared_ptr<MapObstacle>> ObstacleMap::getData(float robotX, float robotY){
    //following method needs global coords
    // return findObjectsWithinSquare(this->radius, robotX, robotY);//floats
    std::vector<std::shared_ptr<MapObstacle>> temp;
    for (int i = 0; i < 5; i++)
    {
        MapObstacle m;
        for (int j = 0; j < i; j++)
        {
            m.points.push_back(Vec2{rand() % 21 - 11, rand() % 21 - 11});
        }
        MapObstacle& n = m;
        temp.push_back(std::make_shared<MapObstacle>(n));
    }
    
    return temp; // delete once EnvMap.h is included, use findObjectsWthinSquare
    //std::vector<std::shared_ptr<MapObstacle>> findObjectsWithinRect(float x_lower_left, float y_lower_left, float x_upper_right, float y_upper_right) const;
}

void ObstacleMap::resetObstacleMap()
{
    for (int i = 0; i < size; i++)
    {
        for (int j = 0; j < size; j++)
        {
           ObstacleMap::obstacle_map[i][j] = false;
        }
    }
}

int ObstacleMap::transform(int val, bool direction)
{
    //direction indicates +/- true is plus, false is -
    if(direction)
    {
        return val + (int)(step_size -  fmodf(val, step_size));
    }else
    {
        return val - (int)fmodf(val, step_size);
    }
}

void ObstacleMap::updateObstacleMap()
{
    float robotX = 10.0f;
    float robotY = 10.0f;
    //UNCOMMENT AFTER TESTING
    //getRobotPosition(robotX, robotY); // from EnvMap
    resetObstacleMap();
    std::vector<std::shared_ptr<MapObstacle>> data = getData(robotX, robotY);
    int x, y;
    for (int i = 0; i < data.size(); i++)
    {
        for (int j = 0; j < data[i]->points.size(); j++)
        {
            Vec2 point = data[i]->points[j];
            x = (int)(robotX - point.x + radius/step_size);
            y = (int)(robotY - point.y + radius/step_size);
            //set four points surrounding given point as blocked
            //likely blocked areas will overlap from proxity of points in MapObstacle
            //to do: check if out of bounds
            obstacle_map[transform(y, true)][transform(x, true)] = true;
            obstacle_map[transform(y, true)][transform(x, false)] = true;
            obstacle_map[transform(y, false)][transform(x, true)] = true;
            obstacle_map[transform(y, false)][transform(x, false)] = true;
        }
    }
}

void ObstacleMap::print()
{
    for (int i = 0; i < size; i++)
    {
        for(int j = 0; j < size; j++)
        {
            if(ObstacleMap::obstacle_map[i][j])
            {
                std::cout << "1 ";
            }else
            {
                std::cout << "0 ";
            }
        }
        std::cout << std::endl;
    } 
}

ObstacleMap::ObstacleMap()
{
    updateObstacleMap();
}


int main()
{
    ObstacleMap map = ObstacleMap();
    map.print();
};