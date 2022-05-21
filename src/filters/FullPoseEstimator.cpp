#include "FullPoseEstimator.h"

#include <functional>

namespace filters {
namespace {
using state_t = FullPoseEstimator::state_t;
using action_t = Eigen::Vector2d;
using statespace::NoiseCovMat;
using statespace::NoiseCovMatX;
using statespace::Vectord;
constexpr int numStates = FullPoseEstimator::numStates;

state_t stateFunc(const DiffDriveKinematics& kinematics, double dt, const state_t& x,
				  const action_t& u, const action_t& w) {
	action_t input = u + w; // noise is applied to the input vector
	wheelvel_t wheelVel{input(0), input(1)};
	return kinematics.getNextPose(wheelVel, x, dt);
}

Eigen::Matrix2d processNoiseFunc(const Eigen::Vector2d& inputNoiseGains, const state_t& x,
								 const action_t& u) {
	// TODO: verify this noise model on an actual robot
	Eigen::Vector2d stds = inputNoiseGains.array() * u.array().abs().sqrt();
	Eigen::Matrix2d Q = statespace::createCovarianceMatrix(stds);
	return Q;
}

MultiSensorEKF<numStates, 2, 2, 2> createEKF(const DiffDriveKinematics& kinematics, double dt,
											 const Eigen::Vector2d& inputNoiseGains,
											 const Eigen::Vector2d& gpsStdDev,
											 double headingStdDev) {
	Eigen::VectorXd gpsStdDevX = gpsStdDev;
	Output gpsOutput(
		numStates, 2, 2,
		[](const state_t& x, const Eigen::Vector2d& v) { return x.topRows<2>() + v; },
		NoiseCovMatX(gpsStdDevX, numStates, 2));
	Eigen::VectorXd headingStdDevVec = Eigen::VectorXd::Constant(1, 1, headingStdDev);
	Output headingOutput(
		numStates, 1, 1,
		[](const state_t& x, const Vectord<1>& v) { return x.bottomRows<1>() + v; },
		NoiseCovMatX(headingStdDevVec, numStates, 1));
	NoiseCovMat<numStates, 2, 2> processNoiseCovMat(std::bind(
		processNoiseFunc, inputNoiseGains, std::placeholders::_1, std::placeholders::_2));
	auto stateFn = std::bind(stateFunc, kinematics, dt, std::placeholders::_1,
							 std::placeholders::_2, std::placeholders::_3);
	return MultiSensorEKF<numStates, 2, 2, 2>(stateFn, processNoiseCovMat, dt,
											  {gpsOutput, headingOutput});
											  // TODO: fix compilation assertions
}

} // namespace

FullPoseEstimator::FullPoseEstimator(const Eigen::Vector2d& inputNoiseGains, double wheelBase,
									 double dt, const Eigen::Vector2d& gpsStdDev,
									 double headingStdDev)
	: kinematics(wheelBase), dt(dt),
	  ekf(createEKF(kinematics, dt, inputNoiseGains, gpsStdDev, headingStdDev)) {}

} // namespace filters
