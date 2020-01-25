#include "ObstacleMap.h"
#include <vector>
//#include "EnvMap.h"

int main()
{

  float a = 1.3f;
  float b = 2.2f;
  float c = 3.4f;
  float d = 3.1f;
  float e = 9.5f;
  float f = 9.1f;
  Point proto_obst{a, b};
  Point proto_obst2{c, d};
  Point proto_obst3{e, f};

  std::vector<Point> obstacles;

  obstacles.push_back(proto_obst);
  obstacles.push_back(proto_obst2);
  obstacles.push_back(proto_obst3);

  ObstacleMap map = ObstacleMap();
  map.update(obstacles);
  map.print();
  // test functionality of obstacleMap w/ print debugging
}