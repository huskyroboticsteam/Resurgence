
#include "rospub.h"

#include <map>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_array.hpp>

namespace rospub {

constexpr const char * ROVER_FRAME = "rover";

std::map<PointPub, std::string> point_pub_names = {
  {PointPub::PLAN_VIZ, "plan_viz"},
  {PointPub::CURRENT_POSE, "current_pose"},
  {PointPub::DRIVE_TARGET, "drive_target"},
  {PointPub::PLAN_TARGET, "plan_target"},
  {PointPub::POSE_GRAPH, "pose_graph"}
};

std::map<ArrayPub, std::string> array_pub_names = {
  {ArrayPub::LIDAR_SCAN, "lidar_scan"},
  {ArrayPub::LANDMARKS, "landmarks"}
};

rclcpp::Node *node;

std::map<PointPub, rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr> point_pubs;
std::map<ArrayPub, rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr> array_pubs;

void init() {
  rclcpp::init(0, nullptr);
  node = new rclcpp::Node("husky_pub");

  for (int idx = PointPub::PLAN_VIZ; idx <= PointPub::POSE_GRAPH; idx++)
  {
    PointPub idx_ = static_cast<PointPub>(idx);
    point_pubs[idx_] = node->create_publisher<geometry_msgs::msg::Point>(
        point_pub_names[idx_], 100);
  }

  for (int idx = ArrayPub::LIDAR_SCAN; idx <= ArrayPub::LANDMARKS; idx++)
  {
    ArrayPub idx_ = static_cast<ArrayPub>(idx);
    array_pubs[idx_] = node->create_publisher<geometry_msgs::msg::PoseArray>(
        array_pub_names[idx_], 100);
  }
}

void shutdown() {
  rclcpp::shutdown();
  free(node);
}

void publish(const pose_t &pose, PointPub topic)
{
	auto message = geometry_msgs::msg::Point();
	message.x = pose(0);
	message.y = pose(1);
	message.z = pose(2);
  point_pubs[topic]->publish(message);
}

void publish_array(const points_t &points, ArrayPub topic)
{
	auto message = geometry_msgs::msg::PoseArray();
	message.header.frame_id = ROVER_FRAME; // technically this is in the window frame actually
	// but we'll figure out our transform situation later
	for (point_t l : points) {
		auto p = geometry_msgs::msg::Pose();
		p.position.x = l(0);
		p.position.y = l(1);
		p.position.z = l(2);
		message.poses.push_back(p);
	}
	array_pubs[topic]->publish(message);
}

} // namespace rospub
