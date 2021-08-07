#pragma once

#include <cmath>
#include <memory>
#include <vector>
#include <future>

#include "Util.h"
#include "WorldData.h"
#include "worldmap/GlobalMap.h"
#include "filters/PoseEstimator.h"
#include "filters/RollingAvgFilter.h"
#include "lidar/PointCloudProcessing.h"
#include "simulator/graphics.h"
#include "simulator/utils.h"
#include "simulator/friendly_graph.h"
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

class Autonomous
{
public:
	explicit Autonomous(const std::vector<URCLeg> &urc_targets, double controlHz);
	Autonomous(const std::vector<URCLeg> &urc_targets, double controlHz, const pose_t &startPose);
	// Returns a pair of floats, in heading, speed
	// Accepts current heading of the robot as parameter
	// Gets the target's coordinate
	pose_t getGPSTargetPose() const;
	void autonomyIter();

private:
	std::vector<URCLeg> urc_targets;
	size_t leg_idx; // which of the urc_targets we're currently navigating toward
	pose_t search_target;
	// Gate targets are {NAN, NAN, NAN} if unset and {INF, INF, INF} if reached
	// gate_targets.second(2) is NAN if targets have not been refined with more accurate landmark measurements
	std::pair<pose_t, pose_t> gate_targets;
	bool gate_direction; // `true` if we go through the gate with left_post_id on our left
	PoseEstimator poseEstimator;
	GlobalMap map;
	bool calibrated = false;
	std::vector<pose_t> calibrationPoses{};
	NavState nav_state;
	ControlState control_state;
	int time_since_plan;
	plan_t plan;
	std::future<plan_t> pending_plan;
	std::future<transform_t> pending_solve;
	double plan_cost;
	pose_t plan_base;
	int plan_idx;
	double search_theta_increment;
	int mapLoopCounter; // number of times the map has been updated
	int mapBlindPeriod; // the number of loops to wait before starting to build the map
	bool mapDoesOverlap;
	unsigned int mapOverlapSampleThreshold; // at least these many points required to overlap map

	/* Variables for pose graph localization */
	FriendlyGraph pose_graph;
	int pose_id; // counter for how many poses we've added to the graph
	transform_t prev_odom; // odom measurement at the time of the most recent pose in the graph
	trajectory_t smoothed_traj; // cached solution to pose graph optimization (robot trajectory)
	points_t smoothed_landmarks; // cached solution to pose graph optimization (AR tags)

	void setNavState(NavState s);
	void update_nav_state(const pose_t &pose, const pose_t &plan_target);
	pose_t choose_plan_target(const pose_t &pose);
	int getPostID(bool left);
	point_t getPostLocation(bool left);
	bool getPostVisibility(bool left);
	void computeGateTargets(const pose_t &pose, bool choose_direction);
	void updateSearchTarget();
	void endCurrentLeg();
	transform_t optimizePoseGraph(transform_t current_gps, transform_t current_odom);

	double getLinearVel(const pose_t &drive_target, const pose_t &pose, double thetaErr) const;
	double getThetaVel(const pose_t &drive_target, const pose_t &pose, double &thetaErr) const;
	pose_t poseToDraw(const pose_t &pose, const pose_t &current_pose) const;

};
