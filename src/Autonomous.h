#pragma once

#include <cmath>
#include <memory>
#include <vector>
#include <queue>
#include <future>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_array.hpp>

#include "Util.h"
#include "WorldData.h"
#include "worldmap/GlobalMap.h"
#include "filters/PoseEstimator.h"
#include "filters/RollingAvgFilter.h"
#include "lidar/PointCloudProcessing.h"
#include "simulator/graphics.h"
#include "simulator/utils.h"
#include "planning/plan.h"

enum ControlState {
	NEAR_TARGET_POSE,
	FAR_FROM_TARGET_POSE
};

enum NavState {
	GPS,
	POST_VISIBLE,
	SEARCH_PATTERN,
	SEARCH_PATTERN_SECOND_POST,
	GATE_ALIGN,
	GATE_TRAVERSE,
	DONE
};

constexpr std::array<const char *, 7> NAV_STATE_NAMES ({
	"GPS",
	"POST_VISIBLE",
	"SEARCH_PATTERN",
	"SEARCH_PATTERN_SECOND_POST",
	"GATE_ALIGN",
	"GATE_TRAVERSE",
	"DONE"
});

class Autonomous : rclcpp::Node
{
public:
	explicit Autonomous(const std::queue<URCLeg> &urc_targets, double controlHz);
	Autonomous(const std::queue<URCLeg> &urc_targets, double controlHz, const pose_t &startPose);
	// Returns a pair of floats, in heading, speed
	// Accepts current heading of the robot as parameter
	// Gets the target's coordinate
	pose_t getGPSTargetPose() const;
	void autonomyIter();

private:
	std::queue<URCLeg> urc_targets;
	pose_t search_target;
	// Gate targets are {NAN, NAN, NAN} if unset and {INF, INF, INF} if reached
	// gate_targets.second(2) is NAN if targets have not been refined with more accurate landmark measurements
	std::pair<pose_t, pose_t> gate_targets;
	PoseEstimator poseEstimator;
	GlobalMap map;
	bool calibrated = false;
	std::vector<pose_t> calibrationPoses{};
	RollingAvgFilter<5,3> leftPostFilter;
	RollingAvgFilter<5,3> rightPostFilter;
	NavState nav_state;
	ControlState control_state;
	int time_since_plan;
	plan_t plan;
	std::future<plan_t> pending_plan;
	double plan_cost;
	pose_t plan_base;
	int plan_idx;
	double search_theta_increment;
	int mapLoopCounter; // number of times the map has been updated
	int mapBlindPeriod; // the number of loops to wait before starting to build the map
	bool mapDoesOverlap;
	unsigned int mapOverlapSampleThreshold; // at least these many points required to overlap map
	rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr plan_pub;
	rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr curr_pose_pub;
	rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr drive_target_pub;
	rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr plan_target_pub;
	rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr lidar_pub;
	rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr landmarks_pub;

  void setNavState(NavState s);
  void update_nav_state(const pose_t &pose, const pose_t &plan_target);
  pose_t choose_plan_target(const pose_t &pose);
  void updateLandmarkInformation(const transform_t &invTransform);
  void computeGateTargets(const pose_t &pose);
  void updateSearchTarget();
	void endCurrentLeg();

	double getLinearVel(const pose_t &drive_target, const pose_t &pose, double thetaErr) const;
	double getThetaVel(const pose_t &drive_target, const pose_t &pose, double &thetaErr) const;
	pose_t poseToDraw(pose_t &pose, pose_t &current_pose) const;
	void publish(Eigen::Vector3d pose,
			rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr &publisher) const;
	void publish_array(const points_t &points,
			rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr &publisher) const;

};
