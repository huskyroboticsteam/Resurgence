#include "PoseEstimator.h"

typedef Eigen::Matrix3d matrix;
typedef Eigen::Vector3d vector;

PoseEstimator::PoseEstimator(const Eigen::Vector3d &stateStdDevs,
							 const Eigen::Vector3d &measurementStdDevs, double dt)
	: kf(KalmanFilter<3, 3>::createDisc(matrix::Identity(), matrix::Identity(),
										matrix::Identity(), stateStdDevs, measurementStdDevs,
										dt)),
	  dt(dt)
{
}

vector getPoseDiff(const vector &pose, double dt, double thetaVel, double xVel)
{
	double dx = xVel * dt;
	double dTheta = thetaVel * dt;

	vector updated = toPose(toTransformRotateFirst(dx, 0, dTheta) * toTransform(pose), 0);
	return updated - pose;
}

void PoseEstimator::predict(double thetaVel, double xVel)
{
	vector u = getPoseDiff(kf.getState(), dt, thetaVel, xVel);
	kf.predict(u);
}

void PoseEstimator::correct(const transform_t &measurement)
{
	pose_t pose = toPose(measurement, kf.getState()[2]);
	kf.correct(pose);
}

void PoseEstimator::reset(const Eigen::Vector3d &pose)
{
	kf.reset(pose);
}
