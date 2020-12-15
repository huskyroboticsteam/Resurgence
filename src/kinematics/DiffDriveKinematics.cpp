#include "DiffDriveKinematics.h"

DiffDriveKinematics::DiffDriveKinematics(double wheelBaseWidth)
	: wheelBaseWidth(wheelBaseWidth)
{
}

pose_t DiffDriveKinematics::wheelVelToRobotVel(double lVel, double rVel) const
{
	return {(lVel + rVel) / 2.0, 0, (rVel - lVel) / wheelBaseWidth};
}

wheelvel_t DiffDriveKinematics::robotVelToWheelVel(double xVel, double thetaVel) const
{
	return {xVel - 0.5 * wheelBaseWidth * thetaVel, xVel + 0.5 * wheelBaseWidth * thetaVel};
}

pose_t DiffDriveKinematics::getLocalPoseUpdate(double lVel, double rVel, double dt) const
{
	double forward = (lVel + rVel) * 0.5 * dt; // distance the robot drove along a circle
	double dTheta = (rVel - lVel) / wheelBaseWidth * dt; // the change in heading

	double dx, dy;
	// if dTheta is close to 0 use the taylor approximations
	if (abs(dTheta) <= 1e-5)
	{
		dx = forward * (1.0 - dTheta * dTheta / 6.0);
		dy = forward * 0.5 * dTheta;
	}
	else
	{
		dx = forward * sin(dTheta) / dTheta;
		dy = forward * (1 - cos(dTheta)) / dTheta;
	}

	return {dx, dy, dTheta};
}

pose_t DiffDriveKinematics::getLocalPoseUpdate(const wheelvel_t &wheelVel, double dt) const
{
	return getLocalPoseUpdate(wheelVel.lVel, wheelVel.rVel, dt);
}

pose_t DiffDriveKinematics::getPoseUpdate(const wheelvel_t &wheelVel, double heading,
										  double dt) const
{
	return toTransformRotateFirst(0, 0, heading) * getLocalPoseUpdate(wheelVel, dt);
}

pose_t DiffDriveKinematics::getNextPose(const wheelvel_t &wheelVel, const pose_t &pose,
										double dt) const
{
	return pose + getPoseUpdate(wheelVel, pose(2), dt);
}
