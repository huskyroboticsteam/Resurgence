#ifndef ROVER_DRIVETOGATENOCOMPASS_H
#define ROVER_DRIVETOGATENOCOMPASS_H

#include "../simulator/utils.h"
#include "CommandBase.h"

class DriveToGateNoCompass : CommandBase
{
public:
	DriveToGateNoCompass(double driveDist, double angleKP, double vel, transform_t odom, point_t target);
	void update(const transform_t &odom, const transform_t &gps, const pose_t &currPose, const point_t &leftPost, const point_t &rightPost);
	bool isDone() override;
	command_t getOutput() override;

private:
	enum State
	{
		Start,
		DriveForward,
		TurnToTarget,
		Done
	};

	void transitionStates();
	// only valid when state == State::TurnToTarget
	double calculateHeadingErr();

	State state;
	transform_t referenceTransform;
	transform_t currOdom;
	pose_t currPose;
	points_t calibrationPoints;
	double calibrateDriveDist;
	point_t targetPoint;
	double angleKP;
	double vel;
};

#endif // ROVER_DRIVETOGATENOCOMPASS_H