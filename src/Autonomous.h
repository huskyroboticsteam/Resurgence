#pragma once

#include <cmath>
#include <memory>
#include <vector>

#include "Pathfinding/ObstacleMap.h"
#include "Pathfinding/Pather2.h"
#include "Util.h"
#include "WorldData.h"
#include "filters/PoseEstimator.h"
#include "filters/RollingAvgFilter.h"
#include "lidar/PointCloudProcessing.h"
#include "simulator/utils.h"

enum NavState {
	INIT,
	NEAR_TARGET_POSE
};

class Autonomous
{
public:
	explicit Autonomous(const URCLeg &target, double controlHz);
	Autonomous(const URCLeg &target, double controlHz, const pose_t &startPose);
	// Returns a pair of floats, in heading, speed
	// Accepts current heading of the robot as parameter
	// Gets the target's coordinate
	pose_t getTargetPose() const;
	void autonomyIter();

private:
	URCLeg target;
	PoseEstimator poseEstimator;
	bool calibrated = false;
	std::vector<pose_t> calibrationPoses{};
	RollingAvgFilter<5,3> landmarkFilter;
	NavState state;

	// determine direction for robot at any given iteration
	double pathDirection(const points_t &lidar, const pose_t &gpsPose);
	double angleToTarget(const pose_t &gpsPose) const;
	bool arrived(const pose_t &pose) const;

	double getLinearVel(const pose_t &target, const pose_t &pose, double &thetaErr);
	double getThetaVel(const pose_t &target, const pose_t &pose, double &thetaErr);

	ObstacleMap obsMap;
	Pather2 pather;
};
