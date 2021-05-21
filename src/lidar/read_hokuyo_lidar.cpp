
#include "PointCloudProcessing.h"
#include "PointGenerator.h"
#include "URGLidar.h"
#include "../simulator/world_interface.h"
#include "../simulator/utils.h"

// This website may be helpful
// https://sourceforge.net/p/urgnetwork/wiki/top_en/

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
  point_t origin({0,0,1});
  for (int i = 0; i < polarPts.size(); i++)
  {
    pts[i] = lidar::polarToCartesian2(polarPts[i]);
    if ((pts[i] - origin).norm() < 0.015)
    {
      // We're inside lidar min range, so this garbage data.
      // We don't want to include this hit because otherwise the robot
      // will think it's in collision with an obstacle.
      pts[i](2) = 0.0;
    }
  }

  return pts;
}
