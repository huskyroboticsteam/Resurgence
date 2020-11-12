#include "PoseEstimator.h"

typedef Eigen::Matrix3d matrix;
typedef Eigen::Vector3d vector;

pose_t stateFunc(const pose_t &x, const Eigen::Vector2d &u)
{
	double thetaVel = u[0];
	double xVel = u[1];

	double theta = x[2];

	pose_t vel;
	vel << xVel * cos(theta), xVel * sin(theta), thetaVel;
	return vel;
}

pose_t measurementFunc(const pose_t &x)
{
	return x;
}

PoseEstimator::PoseEstimator(const Eigen::Vector3d &stateStdDevs,
							 const Eigen::Vector3d &measurementStdDevs, double dt)
	: ekf(stateFunc, measurementFunc, stateStdDevs, measurementStdDevs, dt), dt(dt)
{
}

void PoseEstimator::predict(double thetaVel, double xVel)
{
	Eigen::Vector2d u;
	u << thetaVel, xVel;
	ekf.predict(u);
}

void PoseEstimator::correct(const transform_t &measurement)
{
	pose_t pose = toPose(measurement, ekf.getState()[2]);
	ekf.correct(pose);
}

void PoseEstimator::reset(const Eigen::Vector3d &pose)
{
	ekf.reset(pose);
}

Eigen::Matrix3d PoseEstimator::getEstimateCovarianceMat() const
{
	return ekf.getEstimateCovarianceMat();
}

pose_t PoseEstimator::getPose() const
{
	return ekf.getState();
}
