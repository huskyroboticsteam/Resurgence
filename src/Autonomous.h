#pragma once

#include "Util.h"
#include "filters/PoseEstimator.h"
#include "filters/RollingAvgFilter.h"
#include "filters/pose_graph/friendly_graph.h"
#include "navtypes.h"
#include "planning/plan.h"
#include "world_interface/data.h"
#include "worldmap/GlobalMap.h"

#include <cmath>
#include <future>
#include <map>
#include <memory>
#include <optional>
#include <vector>

enum ControlState { NEAR_TARGET_POSE, FAR_FROM_TARGET_POSE };

enum NavState {
	GPS,
	POST_VISIBLE,
	SEARCH_PATTERN,
	SEARCH_PATTERN_SECOND_POST,
	GATE_ALIGN,
	GATE_TRAVERSE,
	DONE
};

constexpr std::array<const char*, 7> NAV_STATE_NAMES({"GPS", "POST_VISIBLE", "SEARCH_PATTERN",
													  "SEARCH_PATTERN_SECOND_POST",
													  "GATE_ALIGN", "GATE_TRAVERSE", "DONE"});

class Autonomous {
public:
	explicit Autonomous(const std::vector<navtypes::URCLeg>& urc_targets, double controlHz);
	Autonomous(const std::vector<navtypes::URCLeg>& urc_targets, double controlHz,
			   const navtypes::pose_t& startPose);
	// Returns a pair of floats, in heading, speed
	// Accepts current heading of the robot as parameter
	// Gets the target's coordinate
	navtypes::pose_t getGPSTargetPose() const;
	void autonomyIter();

private:
	std::vector<navtypes::URCLeg> urc_targets;
	size_t leg_idx; // which of the urc_targets we're currently navigating toward
	navtypes::pose_t search_target;
	// Gate targets are {NAN, NAN, NAN} if unset and {INF, INF, INF} if reached
	// gate_targets.second(2) is NAN if targets have not been refined with more accurate
	// landmark measurements
	std::pair<navtypes::pose_t, navtypes::pose_t> gate_targets;
	bool gate_direction; // `true` if we go through the gate with left_post_id on our left
	filters::PoseEstimator poseEstimator;
	GlobalMap map;
	bool calibrated = false;
	std::vector<navtypes::pose_t> calibrationPoses{};
	NavState nav_state;
	ControlState control_state;
	int time_since_plan;
	plan_t plan;
	std::future<plan_t> pending_plan;
	std::future<navtypes::transform_t> pending_solve;
	double plan_cost;
	navtypes::pose_t plan_base;
	int plan_idx;
	double search_theta_increment;
	int mapLoopCounter; // number of times the map has been updated
	int mapBlindPeriod; // the number of loops to wait before starting to build the map
	bool mapDoesOverlap;
	unsigned int
		mapOverlapSampleThreshold; // at least these many points required to overlap map

	std::optional<datatime_t> lastLidarTime;	 // last timestamp of lidar data
	std::optional<datatime_t> lastGPSTime;		 // last timestamp of gps data
	std::map<int, datatime_t> lastLandmarkTimes; // maps landmark idx -> last datapoint time

	/* Variables for pose graph localization */
	filters::pose_graph::FriendlyGraph pose_graph;
	int pose_id; // counter for how many poses we've added to the graph
	navtypes::transform_t
		prev_odom; // odom measurement at the time of the most recent pose in the graph
	navtypes::trajectory_t
		smoothed_traj; // cached solution to pose graph optimization (robot trajectory)
	navtypes::points_t
		smoothed_landmarks; // cached solution to pose graph optimization (AR tags)

	void setNavState(NavState s);
	void update_nav_state(const navtypes::pose_t& pose, const navtypes::pose_t& plan_target);
	navtypes::pose_t choose_plan_target(const navtypes::pose_t& pose);
	navtypes::pose_t getDriveTargetFromPlan(const navtypes::pose_t& pose,
											const navtypes::pose_t& plan_target,
											const navtypes::points_t& lidar_scan);
	int getPostID(bool left);
	navtypes::point_t getPostLocation(bool left, bool verbose = true);
	bool getPostVisibility(bool left);
	void computeGateTargets(const navtypes::pose_t& pose, bool choose_direction);
	void updateSearchTarget();
	void endCurrentLeg();
	navtypes::transform_t optimizePoseGraph(navtypes::transform_t current_odom);

	double getLinearVel(const navtypes::pose_t& drive_target, const navtypes::pose_t& pose,
						double thetaErr) const;
	double getThetaVel(const navtypes::pose_t& drive_target, const navtypes::pose_t& pose,
					   double& thetaErr) const;
	navtypes::pose_t poseToDraw(const navtypes::pose_t& pose,
								const navtypes::pose_t& current_pose) const;
};
