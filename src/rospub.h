
#pragma once

#include "Networking/json.hpp"
#include "simulator/utils.h"

namespace rospub {

enum PointPub { PP_INVALID, PLAN_VIZ, CURRENT_POSE, DRIVE_TARGET, PLAN_TARGET, POSE_GRAPH };

enum ArrayPub { AP_INVALID, LIDAR_SCAN, LANDMARKS };

NLOHMANN_JSON_SERIALIZE_ENUM(PointPub, {{PP_INVALID, nullptr},
										{PLAN_VIZ, "plan_viz"},
										{CURRENT_POSE, "current_pose"},
										{DRIVE_TARGET, "drive_target"},
										{PLAN_TARGET, "plan_target"},
										{POSE_GRAPH, "pose_graph"}})

NLOHMANN_JSON_SERIALIZE_ENUM(ArrayPub, {{AP_INVALID, nullptr},
										{LIDAR_SCAN, "lidar_scan"},
										{LANDMARKS, "landmarks"}})

void init();
void shutdown();
void publish(const pose_t& pose, PointPub topic); // Also can be used for point_t
void publish_array(const points_t& points, ArrayPub topic);

} // namespace rospub
