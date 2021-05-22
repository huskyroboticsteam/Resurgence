#pragma once

#include <cmath>
#include <memory>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_array.hpp>

#include "Util.h"
#include "WorldData.h"
#include "filters/PoseEstimator.h"
#include "filters/RollingAvgFilter.h"
#include "lidar/PointCloudProcessing.h"
#include "simulator/graphics.h"
#include "simulator/utils.h"
#include "planning/plan.h"

enum NavState {
	INIT,
	NEAR_TARGET_POSE,
	SEARCH_PATTERN,
	GATE_ALIGN
};

class Autonomous : rclcpp::Node
{
public:
	explicit Autonomous(const URCLeg &target, double controlHz);
	Autonomous(const URCLeg &target, double controlHz, const pose_t &startPose);
	// Returns a pair of floats, in heading, speed
	// Accepts current heading of the robot as parameter
	// Gets the target's coordinate
	pose_t getGPSTargetPose() const;
	void autonomyIter();

private:
	URCLeg target;
	pose_t search_target;
	// Gate targets are {NAN, NAN, NAN} if unset and {INF, INF, INF} if reached
	// gate_targets.second(2) is NAN if targets have not been refined with more accurate landmark measurements
	std::pair<pose_t, pose_t> gate_targets;
	PoseEstimator poseEstimator;
	bool calibrated = false;
	std::vector<pose_t> calibrationPoses{};
	RollingAvgFilter<5,3> leftPostFilter;
	RollingAvgFilter<5,3> rightPostFilter;
	NavState state;
	int time_since_plan;
	plan_t plan;
	double plan_cost;
	pose_t plan_base;
	int plan_idx;
	double search_theta_increment;
	bool already_arrived;
	rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr plan_pub;
	rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr curr_pose_pub;
	rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr next_pose_pub;
	rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr gate_pub;
	rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr lidar_pub;
	rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr landmarks_pub;

	// determine direction for robot at any given iteration
	double pathDirection(const points_t &lidar, const pose_t &gpsPose);
	double angleToTarget(const pose_t &gpsPose) const;
	bool arrived(const pose_t &pose) const;

	double getLinearVel(const pose_t &target, const pose_t &pose, double thetaErr) const;
	double getThetaVel(const pose_t &target, const pose_t &pose, double &thetaErr) const;
	pose_t poseToDraw(pose_t &pose, pose_t &current_pose) const;
	void publish(Eigen::Vector3d pose,
			rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr &publisher) const;
	void publish_array(const points_t &points,
			rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr &publisher) const;

};
