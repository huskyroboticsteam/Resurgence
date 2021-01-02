#include <Eigen/Core>
#include <iostream>

#include <catch2/catch.hpp>

#include "../../src/filters/ExtendedKalmanFilter.h"
#include "../../src/kinematics/DiffDriveKinematics.h"

using namespace Catch::literals;

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
} // namespace

TEST_CASE("EKF Predict-only")
{
	Eigen::Vector3d std = {0, 0, 0};
	ExtendedKalmanFilter<3, 2, 3, 3, 3> ekf(
		stateFunc, measurementFunc, NoiseCovMat<3, 3, 2>(std), NoiseCovMat<3, 3, 3>(std), 0.1);

	ekf.reset(std, std);
	pose_t currPose = {0, 0, 0};
	for (int i = 0; i < 100; i++)
	{
		double l = randWheelVel();
		double r = randWheelVel();

		constexpr int steps = 5; // maintain velocity for 0.5 sec
		currPose = kinematics.getNextPose({l, r}, currPose, dt * steps);
		for (int j = 0; j < steps; j++) {
			ekf.predict({l, r});
		}

		REQUIRE((currPose - ekf.getState()).norm() == Approx(0).margin(1e-9));
	}
}
