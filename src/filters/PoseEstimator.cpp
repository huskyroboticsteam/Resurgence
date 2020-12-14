#include "PoseEstimator.h"

// Note: The state vector is defined as [x, y, theta].
// The input vector is defined as [lVel, rVel].

namespace
{
statevec_t stateFunc(double wheelBase, double dt, const statevec_t &x, const Eigen::Vector2d &u,
					 const Eigen::Vector2d &w)
{
	// Instead of using euler integration, we can represent the pose update as a twist
	// and then apply that twist to the current pose. Derived from differential drive kinematics.
	// This will be more accurate than euler integration at slower update rates.
	Eigen::Vector2d input = u + w; // noise is applied to the input vector
	double theta = x(2);

	Eigen::Matrix<double, 3, 2> trf;
	trf << 0.5, 0.5, 0, 0, -1 / wheelBase, 1 / wheelBase;

	// This represents [deltaX, 0, deltaTheta]
	// however, this is "pre-transform". We will now transform it to give a more accurate
	// deltaX and deltaY.
	Eigen::Vector3d localUpdate = trf * input * dt;

	double cosTheta = cos(theta);
	double sinTheta = sin(theta);
	// This matrix transforms from local space to map space.
	Eigen::Matrix3d A;
	A << cosTheta, -sinTheta, 0, sinTheta, cosTheta, 0, 0, 0, 1;

	double dTheta = localUpdate(2);
	Eigen::Matrix3d B;
	// If dTheta is close to 0 use the taylor series approximation
	if (abs(dTheta) <= 1e-9)
	{
		B << 1 - dTheta * dTheta / 6.0, -dTheta / 2.0, 0, dTheta / 2.0,
			1 - dTheta * dTheta / 6.0, 0, 0, 0, 1;
	}
	else
	{
		double sinDTheta = sin(dTheta);
		double cosDTheta = cos(dTheta);
		B << sinDTheta, cosDTheta - 1, 0, 1 - cosDTheta, sinDTheta, 0, 0, 0, dTheta;
		B /= dTheta;
	}

	return x + A * B * localUpdate;
}

Eigen::Vector3d measurementFunc(const statevec_t &x, const statevec_t &v)
{
	return x + v; // noise is applied to the measured position
}
} // namespace

PoseEstimator::PoseEstimator(const Eigen::Vector2d &inputNoiseGains,
							 const Eigen::Vector3d &measurementStdDevs, double wheelBase, double dt)
	: ekf([wheelBase, dt](const statevec_t &x, const Eigen::Vector2d &u,
			   const Eigen::Vector2d &w) { return stateFunc(wheelBase, dt, x, u, w); },
		  measurementFunc,
		  NoiseCovMat<numStates, 2, 2>(
			  [inputNoiseGains](const statevec_t &x, const Eigen::Vector2d &u) {
				  // TODO: verify this noise model on an actual robot
				  Eigen::Vector2d stds = inputNoiseGains.array() * u.array().abs().sqrt();
				  Eigen::Matrix<double, 2, 2> Q = StateSpace::createCovarianceMatrix(stds);
				  return Q;
			  }),
		  NoiseCovMat<numStates, numStates, numStates>(measurementStdDevs), dt),
	  wheelBase(wheelBase),
	  dt(dt)
{
	// define the analytical solutions to the jacobians
	ekf.outputFuncJacobianX = [](const statevec_t &x, const statevec_t &v) {
		Eigen::Matrix<double, numStates, numStates> jacobian =
			Eigen::Matrix<double, numStates, numStates>::Identity();
		return jacobian;
	};
	ekf.outputFuncJacobianV = [](const statevec_t &x, const statevec_t &v) {
		Eigen::Matrix<double, numStates, numStates> jacobian =
			Eigen::Matrix<double, numStates, numStates>::Identity();
		return jacobian;
	};
}

void PoseEstimator::predict(double thetaVel, double xVel)
{
	// convert xVel, thetaVel to wheel velocities
	double lVel = xVel - 0.5 * wheelBase * thetaVel;
	double rVel = xVel + 0.5 * wheelBase * thetaVel;
	Eigen::Vector2d u;
	u << lVel, rVel;
	ekf.predict(u);
}

void PoseEstimator::correct(const transform_t &measurement)
{
	pose_t pose = toPose(measurement, getPose()(2));
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
