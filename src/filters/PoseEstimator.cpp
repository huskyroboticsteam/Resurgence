#include "PoseEstimator.h"

#include "../simulator/constants.h"

const double wheelBase = NavSim::ROBOT_WHEEL_BASE;

statevec_t stateFunc(const statevec_t &x, const Eigen::Vector2d &u)
{
	double theta = x(2, 0);

	double lVel = u(0);
	double rVel = u(1);

	double xVel = (lVel + rVel) / 2.0;
	double thetaVel = (rVel - lVel) / wheelBase;

	statevec_t stateDerivative;
	stateDerivative << xVel * cos(theta), xVel * sin(theta), thetaVel;
	return stateDerivative;
}

Eigen::Vector3d measurementFunc(const statevec_t &x)
{
	return x;
}

Eigen::Matrix<double, numStates, numStates> stateFuncJacobianX(const statevec_t &x,
															   const Eigen::Vector2d &u)
{
	double theta = x(2);
	double lVel = u(0);
	double rVel = u(1);

	Eigen::Matrix<double, numStates, numStates> jacobian;
	jacobian.setZero();
	double sinTheta = sin(theta);
	double cosTheta = cos(theta);
	jacobian(0, 2) = -sinTheta * (lVel + rVel) / 2.0;
	jacobian(1, 2) = cosTheta * (lVel + rVel) / 2.0;

	return jacobian;
}

Eigen::Matrix<double, numStates, 2> stateFuncJacobianU(const statevec_t &x,
													   const Eigen::Vector2d &u)
{
	double theta = x(2);

	double sinTheta = sin(theta);
	double cosTheta = cos(theta);
	Eigen::Matrix<double, numStates, 2> jacobian;
	jacobian << cosTheta / 2.0, cosTheta / 2.0, sinTheta / 2.0, sinTheta / 2.0,
		-1.0 / wheelBase, 1.0 / wheelBase;

	return jacobian;
}

PoseEstimator::PoseEstimator(const Eigen::Vector2d &inputStdDevs,
							 const Eigen::Vector3d &measurementStdDevs, double dt)
	: ekf(stateFunc, measurementFunc, inputStdDevs, measurementStdDevs, dt), dt(dt)
{
	ekf.stateFuncJacobianX = stateFuncJacobianX;
	ekf.stateFuncJacobianU = stateFuncJacobianU;
	ekf.outputFuncJacobian = [](const statevec_t &x) {
		Eigen::Matrix<double, numStates, numStates> jacobian =
			Eigen::Matrix<double, numStates, numStates>::Identity();
		return jacobian;
	};
}

void PoseEstimator::predict(double thetaVel, double xVel)
{
	double lVel = xVel - 0.5 * wheelBase * thetaVel;
	double rVel = xVel + 0.5 * wheelBase * thetaVel;
	Eigen::Vector2d u;
	u << lVel, rVel;
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

void PoseEstimator::reset(const pose_t &pose, const pose_t &stdDevs)
{
	ekf.reset(pose, stdDevs);
}

Eigen::Matrix<double, 3, 3> PoseEstimator::getEstimateCovarianceMat() const
{
	return ekf.getEstimateCovarianceMat();
}

pose_t PoseEstimator::getPose() const
{
	return ekf.getState();
}
