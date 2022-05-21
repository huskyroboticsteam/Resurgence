#pragma once

#include "../kinematics/DiffDriveKinematics.h"
#include "../navtypes.h"
#include "MultiSensorEKF.h"

#include <array>

namespace filters {

class FullPoseEstimator {
public:
	static constexpr int numStates = 3;
	using state_t = statespace::Vectord<numStates>;

	FullPoseEstimator(const Eigen::Vector2d& inputNoiseGains, double wheelBase, double dt,
					  const Eigen::Vector2d& gpsStdDev, double headingStdDev);

	void correctGPS(const navtypes::point_t& gps);

	void correctHeading(double heading);

	/**
	 * @brief Use the model to predict the next system state, given the current inputs.
	 *
	 * @param thetaVel Commanded rotational velocity.
	 * @param xVel Commanded x velocity.
	 */
	void predict(double thetaVel, double xVel);

	/**
	 * @brief Get the current estimate covariance matrix.
	 *
	 * This is an indication of the uncertainty of the pose estimate.
	 *
	 * @return The estimate covariance matrix, AKA the P matrix.
	 */
	Eigen::Matrix<double, numStates, numStates> getEstimateCovarianceMat() const;

	/**
	 * @brief Reset the pose estimator.
	 *
	 * Sets the state estimate to the zero vector and resets the estimate covariance matrix
	 * to a diagonal matrix with large values to reflect complete uncertainty in the current
	 * pose. If you have any information about the current pose, use reset(const pose_t &,
	 * const pose_t &)
	 */
	void reset();

	/**
	 * @brief Reset the pose estimator.
	 *
	 * Sets the state estimate to the supplied vector and resets the estimate covariance matrix
	 * to a diagonal matrix with large values to reflect complete uncertainty in the current
	 * pose. If you have any information about the current pose, use reset(const pose_t &,
	 * const pose_t &)
	 *
	 * @param pose The pose to which the state estimate will be set.
	 */
	void reset(const navtypes::pose_t& pose);

	/**
	 * @brief Reset the pose estimator.
	 *
	 * Sets the state estimate to the supplied vector and sets the estimate covariance matrix
	 * to reflect the given uncertainty.
	 *
	 * @param pose The pose to which the state estimate will be set.
	 * @param stdDevs The standard deviation for each element in the pose.
	 */
	void reset(const navtypes::pose_t& pose, const navtypes::pose_t& stdDevs);

	/**
	 * @brief Gets the current state estimate.
	 *
	 * @return The state estimate.
	 */
	navtypes::pose_t getPose() const;

private:
	DiffDriveKinematics kinematics;
	double dt;
	MultiSensorEKF<numStates, 2, 2, 2> ekf;
};

} // namespace filters
