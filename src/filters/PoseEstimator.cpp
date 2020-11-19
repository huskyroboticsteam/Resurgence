#include "PoseEstimator.h"

#include "../simulator/constants.h"

typedef Eigen::Matrix3d matrix;
typedef Eigen::Vector3d vector;

const double wheelBase = NavSim::ROBOT_WHEEL_BASE;

double rateLimit(double curr, double target, double maxROC, double dt, double &roc)
{
	double maxChange = maxROC * dt;
	if (abs(target - curr) <= maxChange)
	{
		roc = (target - curr) / dt;
		return target;
	}
	else
	{
		int sign = curr < target ? 1 : -1;
		roc = sign * maxROC;
		return curr + sign * maxChange;
	}
}

pose_t stateToPose(const statevec_t &state)
{
	return state.block<3, 1>(0, 0);
}

statevec_t poseToState(const pose_t &pose)
{
	statevec_t state = statevec_t::Zero();
	state.block<3, 1>(0, 0) = pose;
	return state;
}

statevec_t PoseEstimator::stateFunc(const statevec_t &x, const Eigen::Vector2d &u) const
{
	double theta = x(2, 0);

	double lastRVel = x(3, 0) + 0.5 * wheelBase * x(4, 0);
	double lastLVel = x(3, 0) - 0.5 * wheelBase * x(4, 0);

	double targetRVel = u(1) + 0.5 * wheelBase * u(0);
	double targetLVel = u(1) - 0.5 * wheelBase * u(0);

	double rAccel, lAccel;
	double rVel = rateLimit(lastRVel, targetRVel, maxWheelAccel, dt, rAccel);
	double lVel = rateLimit(lastLVel, targetLVel, maxWheelAccel, dt, lAccel);

	double xVel = (lVel + rVel) / 2.0;
	double thetaVel = (rVel - lVel) / wheelBase;

	double xAccel = (rAccel + lAccel) / 2.0;
	double thetaAccel = (rAccel - lAccel) / wheelBase;

	statevec_t stateDerivative;
	stateDerivative << xVel * cos(theta), xVel * sin(theta), thetaVel, xAccel, thetaAccel;
	return stateDerivative;
}

statevec_t PoseEstimator::measurementFunc(const statevec_t &x) const
{
	return x;
}

PoseEstimator::PoseEstimator(const statevec_t &stateStdDevs,
							 const Eigen::Vector3d &measurementStdDevs, double maxWheelAccel,
							 double dt)
	: ekf([this](const statevec_t &x,
				 const Eigen::Vector2d &u) { return this->stateFunc(x, u); },
		  [this](const statevec_t &x) { return this->measurementFunc(x); }, stateStdDevs,
		  poseToState(measurementStdDevs), dt),
	  maxWheelAccel(maxWheelAccel), dt(dt)
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
	statevec_t state = ekf.getState();
	state.block<3,1>(0, 0) = pose;
	ekf.correct(state);
}

void PoseEstimator::reset()
{
	reset(pose_t::Zero());
}

void PoseEstimator::reset(const pose_t &pose)
{
	statevec_t state = poseToState(pose);
	ekf.reset(state);
}

Eigen::Matrix<double, 5, 5> PoseEstimator::getEstimateCovarianceMat() const
{
	return ekf.getEstimateCovarianceMat();
}

pose_t PoseEstimator::getPose() const
{
	return stateToPose(ekf.getState());
}
