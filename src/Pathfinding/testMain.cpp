#include "ObstacleMap.h"
#include "../mapping/EnvMap.h"

int main()
{
    // create and initialize envmap with everything obstacleMap needs to access
    EnvMap envmap;
    envmap.setRobotPosition(0.0f, 0.0f);

    MapObstacle m;
    for (int j = 1; j < 4; j++)
    {
        float x,y;
        envmap.getRobotPosition(x,y);

        m.points.push_back(Vec2{x,y});
    }
    MapObstacle& n = m;
    envmap.newObstacleUID(m);

    // create obstacleMap using envmap
    ObstacleMap map = ObstacleMap(envmap);
    map.print();
    // test functionality of obstacleMap w/ print debugging
}