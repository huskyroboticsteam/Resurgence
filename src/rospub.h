
#pragma once

#include "simulator/utils.h"

namespace rospub {

enum PointPub { PLAN_VIZ, CURRENT_POSE, DRIVE_TARGET, PLAN_TARGET, POSE_GRAPH };

enum ArrayPub { LIDAR_SCAN, LANDMARKS };

void init();
void shutdown();
void publish(const pose_t& pose, PointPub topic); // Also can be used for point_t
void publish_array(const points_t& points, ArrayPub topic);

} // namespace rospub
