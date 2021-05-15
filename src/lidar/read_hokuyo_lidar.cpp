
#include "PointCloudProcessing.h"
#include "PointGenerator.h"
#include "URGLidar.h"
#include "../simulator/utils.h"

bool lidar_initialized = false;
URGLidar urg_lidar;

points_t readLidarScan() {
  if (!lidar_initialized) {
    if (!urg_lidar.open())
    {
      perror("failed to open lidar");
      return {};
    }
    else
    {
      lidar_initialized = true;
    }
  }

  if (!urg_lidar.createFrame())
  {
    perror("failed to create frame");
    return {};
  }

  std::vector<Polar2D> polarPts = urg_lidar.getLastFrame();
  std::vector<point_t> pts(polarPts.size());
  for (Polar2D p : polarPts)
  {
    pts.push_back(lidar::polarToCartesian2(p));
  }

  return pts;
}
