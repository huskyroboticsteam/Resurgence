#include "PoseEstimator.h"

#include "../simulator/constants.h"

const double wheelBase = NavSim::ROBOT_WHEEL_BASE;

statevec_t PoseEstimator::stateFunc(const statevec_t &x, const Eigen::Vector2d &u) const
{
	double theta = x(2, 0);

	double rVel = u(1) + 0.5 * wheelBase * u(0);
	double lVel = u(1) - 0.5 * wheelBase * u(0);

	double xVel = (lVel + rVel) / 2.0;
	double thetaVel = (rVel - lVel) / wheelBase;

	statevec_t stateDerivative;
	stateDerivative << xVel * cos(theta), xVel * sin(theta), thetaVel;
	return stateDerivative;
}

Eigen::Vector3d PoseEstimator::measurementFunc(const statevec_t &x) const
{
	Eigen::Vector3d ret = x.block<3, 1>(0, 0);
	return ret;
}

PoseEstimator::PoseEstimator(const statevec_t &stateStdDevs,
							 const Eigen::Vector3d &measurementStdDevs,
							 double dt)
	: ekf([this](const statevec_t &x,
				 const Eigen::Vector2d &u) { return this->stateFunc(x, u); },
		  [this](const statevec_t &x) { return this->measurementFunc(x); }, stateStdDevs,
		  measurementStdDevs, dt),
	  dt(dt)
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
	pose_t pose = toPose(measurement, getPose()[2]);
	ekf.correct(pose);
}

void PoseEstimator::reset()
{
	reset(pose_t::Zero());
}

void PoseEstimator::reset(const pose_t &pose)
{
	ekf.reset(pose);
}

Eigen::Matrix<double, 3, 3> PoseEstimator::getEstimateCovarianceMat() const
{
	return ekf.getEstimateCovarianceMat();
}

pose_t PoseEstimator::getPose() const
{
	return ekf.getState();
}
