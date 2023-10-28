#include "PoseEstimator.h"

#include "../navtypes.h"
#include "../utils/transform.h"

using namespace navtypes;
using namespace kinematics;

namespace filters {

// Note: The state vector is defined as [x, y, theta].
// The input vector is defined as [lVel, rVel].

namespace {

using statevec_t = PoseEstimator::statevec_t;
using statespace::NoiseCovMat;

statevec_t stateFunc(const kinematics::DiffDriveKinematics& kinematics, double dt,
					 const statevec_t& x, const Eigen::Vector2d& u, const Eigen::Vector2d& w) {
	Eigen::Vector2d input = u + w; // noise is applied to the input vector
	wheelvel_t wheelVel{input(0), input(1)};
	return kinematics.getNextPose(wheelVel, x, dt);
}

Eigen::Vector3d measurementFunc(const statevec_t& x, const statevec_t& v) {
	return x + v; // noise is applied to the measured position
}
} // namespace

PoseEstimator::PoseEstimator(const Eigen::Vector2d& inputNoiseGains,
							 const Eigen::Vector3d& measurementStdDevs, double wheelBase,
							 double dt)
	: ekf(
		  [this](const statevec_t& x, const Eigen::Vector2d& u, const Eigen::Vector2d& w) {
			  return stateFunc(this->PoseEstimator::kinematics, this->dt, x, u, w);
		  },
		  measurementFunc,
		  NoiseCovMat<numStates, 2, 2>(
			  [inputNoiseGains](const statevec_t& x, const Eigen::Vector2d& u) {
				  // TODO: verify this noise model on an actual robot
				  Eigen::Vector2d stds = inputNoiseGains.array() * u.array().abs().sqrt();
				  Eigen::Matrix<double, 2, 2> Q = statespace::createCovarianceMatrix(stds);
				  return Q;
			  }),
		  NoiseCovMat<numStates, numStates, numStates>(measurementStdDevs), dt),
	  kinematics(wheelBase), dt(dt) {
	// define the analytical solutions to the jacobians
	ekf.outputFuncJacobianX = [](const statevec_t& x, const statevec_t& v) {
		Eigen::Matrix<double, numStates, numStates> jacobian =
			Eigen::Matrix<double, numStates, numStates>::Identity();
		return jacobian;
	};
	ekf.outputFuncJacobianV = [](const statevec_t& x, const statevec_t& v) {
		Eigen::Matrix<double, numStates, numStates> jacobian =
			Eigen::Matrix<double, numStates, numStates>::Identity();
		return jacobian;
	};
}

void PoseEstimator::predict(double thetaVel, double xVel) {
	// convert xVel, thetaVel to wheel velocities
	wheelvel_t wheelVels = kinematics.robotVelToWheelVel(xVel, thetaVel);
	Eigen::Vector2d u;
	u << wheelVels.lVel, wheelVels.rVel;
	ekf.predict(u);
}

void PoseEstimator::correct(const transform_t& measurement) {
	pose_t pose = util::toPose(measurement, getPose()(2));
	ekf.correct(pose);
}

void PoseEstimator::reset() {
	reset(pose_t::Zero());
}

void PoseEstimator::reset(const pose_t& pose) {
	ekf.reset(pose);
}

void PoseEstimator::reset(const pose_t& pose, const pose_t& stdDevs) {
	ekf.reset(pose, stdDevs);
}

Eigen::Matrix<double, 3, 3> PoseEstimator::getEstimateCovarianceMat() const {
	return ekf.getEstimateCovarianceMat();
}

pose_t PoseEstimator::getPose() const {
	return ekf.getState();
}

} // namespace filters
