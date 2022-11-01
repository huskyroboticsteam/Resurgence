#include "FullPoseEstimator.h"

#include "../Util.h"

#include <functional>
#include "../navtypes.h"

namespace filters {
namespace {
using namespace navtypes;
using state_t = FullPoseEstimator::state_t;
using action_t = Eigen::Vector2d;
using navtypes::Matrixd;
using statespace::NoiseCovMat;
using statespace::NoiseCovMatX;
using navtypes::Vectord;
constexpr int numStates = FullPoseEstimator::numStates;
constexpr int numSensors = FullPoseEstimator::numSensors;

constexpr int GPS_IDX = 0;
constexpr int HEADING_IDX = 1;

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

const Matrixd<2, numStates> gpsOutputJacobianX(const state_t&, const Eigen::Vector2d&) {
	static_assert(numStates == 3);
	Matrixd<2, numStates> ret;
	ret << 1, 0, 0, 0, 1, 0;
	return ret;
}

const Eigen::Matrix2d gpsOutputJacobianV(const state_t&, const Eigen::Vector2d&) {
	return Eigen::Matrix2d::Identity();
}

const Matrixd<1, 3> headingOutputJacobianX(const state_t&, const Vectord<1>&) {
	Matrixd<1, numStates> ret = Matrixd<1, numStates>::Zero();
	ret(numStates - 1) = 1.0;
	return ret;
}

const Matrixd<1, 1> headingOutputJacobianV(const state_t&, const Vectord<1>&) {
	return Matrixd<1, 1>::Ones();
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
	gpsOutput.setOutputFuncJacobianX(gpsOutputJacobianX);
	gpsOutput.setOutputFuncJacobianV(gpsOutputJacobianV);
	Eigen::VectorXd headingStdDevVec = Eigen::VectorXd::Constant(1, 1, headingStdDev);
	Output headingOutput(
		numStates, 1, 1,
		[](const state_t& x, const Vectord<1>& v) { return x.bottomRows<1>() + v; },
		NoiseCovMatX(headingStdDevVec, numStates, 1));
	headingOutput.setOutputFuncJacobianX(headingOutputJacobianX);
	headingOutput.setOutputFuncJacobianV(headingOutputJacobianV);
	NoiseCovMat<numStates, 2, 2> processNoiseCovMat(std::bind(
		processNoiseFunc, inputNoiseGains, std::placeholders::_1, std::placeholders::_2));
	auto stateFn = std::bind(stateFunc, kinematics, dt, std::placeholders::_1,
							 std::placeholders::_2, std::placeholders::_3);
	return MultiSensorEKF<numStates, 2, 2, numSensors>(stateFn, processNoiseCovMat, dt,
													   {gpsOutput, headingOutput});
}

} // namespace

FullPoseEstimator::FullPoseEstimator(const Eigen::Vector2d& inputNoiseGains, double wheelBase,
									 double dt, const Eigen::Vector2d& gpsStdDev,
									 double headingStdDev)
	: kinematics(wheelBase), dt(dt),
	  ekf(createEKF(kinematics, dt, inputNoiseGains, gpsStdDev, headingStdDev)) {}

void FullPoseEstimator::correctGPS(const point_t& gps) {
	ekf.correct<GPS_IDX>(gps.topRows<2>());
}

void FullPoseEstimator::correctHeading(double heading) {
	double currHeading = ekf.getState()(2);
	heading = util::closestHeading(heading, currHeading);
	Vectord<1> headingVec;
	headingVec << heading;
	ekf.correct<HEADING_IDX>(headingVec);
}

void FullPoseEstimator::predict(double thetaVel, double xVel) {
	// convert xVel, thetaVel to wheel velocities
	wheelvel_t wheelVels = kinematics.robotVelToWheelVel(xVel, thetaVel);
	Eigen::Vector2d u;
	u << wheelVels.lVel, wheelVels.rVel;
	ekf.predict(u);
}

void FullPoseEstimator::reset() {
	reset(pose_t::Zero());
}

void FullPoseEstimator::reset(const pose_t& pose) {
	ekf.reset(pose);
}

void FullPoseEstimator::reset(const pose_t& pose, const pose_t& stdDevs) {
	ekf.reset(pose, stdDevs);
}

Eigen::Matrix<double, 3, 3> FullPoseEstimator::getEstimateCovarianceMat() const {
	return ekf.getEstimateCovarianceMat();
}

pose_t FullPoseEstimator::getPose() const {
	return ekf.getState();
}

} // namespace filters
