#pragma once

#include <cmath>
#include <memory>
#include <vector>

#include "FakeMap.h"
#include "Pathfinding/ObstacleMap.h"
#include "Pathfinding/Pather2.h"
#include "Util.h"
#include "WorldData.h"
#include "filters/PoseEstimator.h"
#include "filters/RollingAvgFilter.h"
#include "lidar/PointCloudProcessing.h"
#include "simulator/utils.h"

class Autonomous
{
public:
	explicit Autonomous(PointXY);
	Autonomous(PointXY target, const pose_t &startPose);
	// Returns a pair of floats, in heading, speed
	// Accepts current heading of the robot as parameter
	std::pair<float, float> getDirections(float currHeading);
	// Gets the target's coordinate
	PointXY getTarget();
	void setWorldData(std::shared_ptr<WorldData>);
	void autonomyIter();

private:
	PointXY target;
	PoseEstimator poseEstimator;
	RollingAvgFilter<5,3> landmarkFilter;
	bool calibrated = false;
	std::vector<pose_t> calibrationPoses{};
	float targetHeading;
	int state;		  // 1 is move forwards, 0 is turning, -1 is back up
	bool rightTurn;	  // boolean for turning right or turning towards target
	int forwardCount; // Counter for how many times to move forwards after a set turn
	std::shared_ptr<WorldData> worldData;
	std::pair<float, float> stateForwards(float currHeading,
										  std::pair<float, float> directions);
	std::pair<float, float> stateTurn(float currHeading, std::pair<float, float> directions);
	std::pair<float, float> stateBackwards(float currHeading,
										   std::pair<float, float> directions);

	// determine direction for robot at any given iteration
	double pathDirection(const points_t &lidar, const pose_t &gpsPose);
	double angleToTarget(const pose_t &gpsPose) const;
	bool arrived(const pose_t &pose) const;

	// helpers to use simulator utils types
	PointXY point_tToPointXY(const point_t &pnt) const;
	std::vector<PointXY> points_tToPointXYs(const points_t &pnts) const;

	ObstacleMap obsMap;
	Pather2 pather;
};
