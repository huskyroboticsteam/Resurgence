#pragma once

#include "../../simulator/utils.h"
#include "../CommandBase.h"

class DriveThroughGate : CommandBase
{
public:
	DriveThroughGate(double thetaKP, double slowVel, double fastVel);

	void reset(transform_t &odom);

	void update(const transform_t &odom, const point_t &left, const point_t &right);

	bool isDone() override;
	command_t getOutput() override;

	void transitionStates();

private:
	enum State
	{
		Start,
		OneVisibleTurn,
		OneVisibleDrive,
		BothVisible,
		Align,
		DriveThrough,
		Done
	};

	transform_t startOdom;
	transform_t currOdom;
	point_t leftPost, rightPost;
	pose_t driveTarget; // always in "global space"
	double thetaKP;
	double slowVel;
	double fastVel;
	State state;

	// gets the transform relative to the starting odometry
	transform_t getTrfFromStart();

	command_t getCommandToTarget(const transform_t &trf, const point_t &target);
};
