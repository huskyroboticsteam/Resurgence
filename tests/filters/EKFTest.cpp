#include <catch2/catch.hpp>

#include "../../src/filters/ExtendedKalmanFilter.h"
#include "../../src/kinematics/DiffDriveKinematics.h"

using namespace Catch::literals;
using namespace filters;

namespace
{
const DiffDriveKinematics kinematics(1);
constexpr double dt = 0.1;

pose_t stateFunc(const pose_t &x, const Eigen::Vector2d &u, const pose_t &w)
{
	pose_t pos = x + w;
	return kinematics.getNextPose({u(0), u(1)}, pos, dt);
}

pose_t measurementFunc(const pose_t &x, const pose_t &v)
{
	return x + v;
}

double randWheelVel()
{
	return -5 + (rand() / (RAND_MAX / 10.0)); // random number between -5 and 5
}

pose_t randPose()
{
	return {randWheelVel(), randWheelVel(), randWheelVel()};
}
} // namespace

TEST_CASE("EKF Predict-only")
{
	Eigen::Vector3d eps = {1e-3, 1e-3, 1e-3};
	ExtendedKalmanFilter<3, 2, 3, 3, 3> ekf(
		stateFunc, measurementFunc, NoiseCovMat<3, 3, 2>(eps), NoiseCovMat<3, 3, 3>(eps), 0.1);

	pose_t currPose = {0, 0, 0};
	ekf.reset(currPose, eps);
	for (int i = 0; i < 100; i++)
	{
		double l = randWheelVel();
		double r = randWheelVel();

		constexpr int steps = 5; // maintain velocity for 0.5 sec
		currPose = kinematics.getNextPose({l, r}, currPose, dt * steps);
		for (int j = 0; j < steps; j++)
		{
			ekf.predict({l, r});
		}

		REQUIRE((currPose - ekf.getState()).norm() == Approx(0).margin(1e-9));
	}
}

TEST_CASE("EKF Correct-only")
{
	Eigen::Vector3d eps = {1e-3, 1e-3, 1e-3};
	ExtendedKalmanFilter<3, 2, 3, 3, 3> ekf(
		stateFunc, measurementFunc, NoiseCovMat<3, 3, 2>(eps), NoiseCovMat<3, 3, 3>(eps), 0.1);

	Eigen::Vector3d zero = {0, 0, 0};

	for (int i = 0; i < 100; i++)
	{
		ekf.reset(zero);
		pose_t pose = randPose();
		ekf.correct(pose);
		REQUIRE((ekf.getState() - pose).norm() == Approx(0).margin(1e-4));
	}
}
