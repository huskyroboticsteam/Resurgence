#include "ObstacleMap.h"
#include <vector>
//#include "EnvMap.h"

int main()
{
    // create and initialize envmap with everything obstacleMap needs to access
    //EnvMap envmap;
    //envmap.setRobotPosition(0.0f, 0.0f);

    Point proto_obst{3, 4};
    Point proto_obst2{8, 9};
    Point proto_obst3{3, 7};

    std::vector<Point> obstacles;

    //obstacles.push_back(proto_obst);
    //obstacles.push_back(proto_obst2 );
   // obstacles.push_back(proto_obst3);

     //for(int i = 0; i < obstacles.size; i++){
     //    System.out.print(obstacles[i]*.x);
    // }

    //

    // proto_obst.position.x = 1.0f;
    // proto_obst.position.y = - 1.0f;
    // size_t obst_uid_1 = envmap.newObstacleUID(proto_obst);

    // proto_obst.position.x = - 1.5f;
    // proto_obst.position.y = - 6.0f;
    // size_t obst_uid_2 = envmap.newObstacleUID(proto_obst);

    // std::shared_ptr<MapObstacle> obst_1_ptr = map.getObstacle(obst_uid_1);
    // std::shared_ptr<MapObstacle> obst_2_ptr = map.getObstacle(obst_uid_2);

    // create obstacleMap using envmap
    
   // ObstacleMap map = ObstacleMap();
    //map.update(obstacles);
    //map.print();
    // test functionality of obstacleMap w/ print debugging
}