#pragma once

#include <Eigen/Core>

class PoseEstimator
{
public:
	PoseEstimator(const Eigen::Vector3d &stateStdDevs,
				  const Eigen::Vector3d &measurementStdDevs, double dt);

	void correct(const Eigen::Vector3d &gps);
	void predict(double thetaVel, double xVel);

	Eigen::Vector3d xHat;

private:
	double dt;
	Eigen::Matrix3d stateCovariance;
	Eigen::Matrix3d measurementCovariance;
	Eigen::Matrix3d A, B, C, D;
	Eigen::Matrix3d gainMatrix;
};
