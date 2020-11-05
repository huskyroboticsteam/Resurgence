#include "PoseEstimator.h"

typedef Eigen::Matrix3d matrix;
typedef Eigen::Vector3d vector;

pose_t stateFunc(const pose_t &x, const Eigen::Vector2d &u)
{
	double dTheta = u[0];
	double dx = u[1];

	return toPose(toTransformRotateFirst(dx, 0, dTheta) * toTransform(x), x[2]);
}

pose_t measurementFunc(const pose_t &x)
{
	return x;
}

matrix stateFuncJacobian(const pose_t &pose, const Eigen::Vector2d &u)
{
	double dTheta = u[0];
	double dx = u[1];

	double theta = pose[2];

	matrix jacobian;
	jacobian << -1, 0, -dx * sin(theta + dTheta), 0, -1, dx * cos(theta + dTheta), 0, 0, 1;
	return jacobian;
}

matrix measurementFuncJacobian(const pose_t &x) {
	return matrix::Identity();
}

PoseEstimator::PoseEstimator(const Eigen::Vector3d &stateStdDevs,
							 const Eigen::Vector3d &measurementStdDevs, double dt)
	: ekf(stateFunc, measurementFunc, stateFuncJacobian, measurementFuncJacobian, stateStdDevs, measurementStdDevs),
	  dt(dt)
{
}

void PoseEstimator::predict(double thetaVel, double xVel)
{
	Eigen::Vector2d u;
	u << thetaVel, xVel;
	u *= dt;
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
