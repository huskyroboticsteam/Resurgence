#include "PoseEstimatorLinear.h"

namespace {
typedef Eigen::Matrix3d matrix;
typedef Eigen::Vector3d vector;

vector getPoseDiff(const vector& pose, double dt, double thetaVel, double xVel) {
	double dx = xVel * dt;
	double dTheta = thetaVel * dt;

	vector updated =
		toPose(toTransformRotateFirst(dx, 0, dTheta) * toTransform(pose), pose(2));
	return updated - pose;
}
} // namespace

PoseEstimatorLinear::PoseEstimatorLinear(const Eigen::Vector3d& stateStdDevs,
										 const Eigen::Vector3d& measurementStdDevs, double dt)
	: kf(KalmanFilter<3, 3, 3>::createDiscrete(matrix::Identity(), matrix::Identity(),
											   matrix::Identity(), stateStdDevs,
											   measurementStdDevs, dt)),
	  dt(dt) {
}

void PoseEstimatorLinear::predict(double thetaVel, double xVel) {
	vector u = getPoseDiff(kf.getState(), dt, thetaVel, xVel);
	kf.predict(u);
}

void PoseEstimatorLinear::correct(const transform_t& measurement) {
	pose_t pose = toPose(measurement, kf.getState()(2));
	kf.correct(pose);
}

void PoseEstimatorLinear::reset(const Eigen::Vector3d& pose) {
	kf.reset(pose);
}

void PoseEstimatorLinear::reset(const pose_t& pose, const pose_t& stdDevs) {
	kf.reset(pose, stdDevs);
}
