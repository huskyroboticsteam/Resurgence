#include "ObstacleMap.h"
#include "../mapping/EnvMap.h"

int main()
{
    // create and initialize envmap with everything obstacleMap needs to access
    EnvMap envmap;
    envmap.setRobotPosition(0.0f, 0.0f);

    MapObstacle proto_obst;
    proto_obst.position.x = 0.0f;
    proto_obst.position.y = 0.0f;
    size_t obst_uid_1 = envmap.newObstacleUID(proto_obst);

    proto_obst.position.x = - 1.5f;
    proto_obst.position.y = - 6.0f;
    size_t obst_uid_2 = envmap.newObstacleUID(proto_obst);
    
   
    // create obstacleMap using envmap
    ObstacleMap map = ObstacleMap(envmap);
    map.print();
    // test functionality of obstacleMap w/ print debugging
}