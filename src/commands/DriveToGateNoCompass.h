#pragma once

#include "../simulator/utils.h"
#include "CommandBase.h"

class DriveToGateNoCompass : CommandBase {
public:
	DriveToGateNoCompass(double driveDist, double angleKP, double vel, double initial_heading);
	void reset(transform_t& odom, point_t& target);
	void update(const transform_t& odom, const transform_t& gps, const pose_t& currPose,
				const point_t& leftPost, const point_t& rightPost);
	bool isDone() override;
	bool isAlmostDone();
	void setDone(const transform_t& odom);
	command_t getOutput() override;

private:
	enum State { Start, DriveForward, TurnToTarget, AlmostDone, Done };

	void transitionStates();
	void checkpoint();
	double distFromCheckpoint();
	double headingChangeSinceCheckpoint();
	double getCurrentHeading();
	double getTargetHeading();
	double calculateHeadingErr();

	State state;
	transform_t checkpointOdom;
	double checkpointHeading;
	transform_t currOdom;
	pose_t currPose;
	points_t calibrationPoints;
	double calibrateDriveDist;
	point_t targetPoint;
	double angleKP;
	double vel;
};
