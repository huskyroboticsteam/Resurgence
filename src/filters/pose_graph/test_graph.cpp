
#include <iostream>
#include <cmath>
#include "graph.h"
#include "factors.h"

using namespace filters::pose_graph;

int main() {
  values x0(3);
  x0 << 0.1, 2.0, 4.0;

  Graph g;
  g.add(new GPSFactor(0,     0.1, 0.0));
  g.add(new GPSFactor(1,     0.1, 2.0));
  g.add(new GPSFactor(2,     0.1, 4.0));
  g.add(new OdomFactor(0, 1, 0.2, 2.0));
  g.add(new OdomFactor(1, 2, 0.2, 2.0));
  g.solve(x0);
  std::cout << g.solution() << std::endl;
  std::cout << g.covariance() << std::endl;

  return 0;
}
