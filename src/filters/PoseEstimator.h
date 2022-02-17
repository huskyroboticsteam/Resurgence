#pragma once

#include "../kinematics/DiffDriveKinematics.h"
#include "../simulator/utils.h"
#include "ExtendedKalmanFilter.h"

#include <Eigen/Core>

namespace filters {

constexpr int numStates = 3;
using statevec_t = Eigen::Matrix<double, numStates, 1>;

/**
 * This class uses a Kalman Filter to continuously estimate the pose of the robot in 2d space.
 * The tracked states are x, y, and heading. All of these states are in map space.
 */
class PoseEstimator {
public:
	/**
	 * Create a new Pose Estimator.
	 *
	 * @param inputNoiseGains The gain for the noise in each dimension of the input space.
	 * This pose estimator models process noise as noise applied to the wheel velocities
	 * with standard deviation proportional to the wheel velocity. The standard deviation of
	 * the noise is modelled as k * sqrt(abs(v)) where k is the input noise gain and v is the
	 * wheel velocity.
	 * @param measurementStdDevs The standard deviations for each of the measurement elements.
	 * 							 This represents additive noise in the measurements.
	 * @param wheelBase The distance between the left and right wheels.
	 * @param dt The time in seconds between updates.
	 */
	PoseEstimator(const Eigen::Vector2d& inputNoiseGains,
				  const Eigen::Vector3d& measurementStdDevs, double wheelBase, double dt);

	/**
	 * Correct the pose estimation with measurement data.
	 * The measurement should be in the same space as the state.
	 *
	 * @param measurement The measurement to use to correct the filter, as a transform.
	 */
	void correct(const transform_t& measurement);

	/**
	 * Use the model to predict the next system state, given the current inputs.
	 *
	 * @param thetaVel Commanded rotational velocity.
	 * @param xVel Commanded x velocity.
	 */
	void predict(double thetaVel, double xVel);

	/**
	 * Get the current estimate covariance matrix.
	 * This is an indication of the uncertainty of the pose estimate.
	 *
	 * @return The estimate covariance matrix, AKA the P matrix.
	 */
	Eigen::Matrix<double, numStates, numStates> getEstimateCovarianceMat() const;

	/**
	 * Sets the state estimate to the zero vector and resets the estimate covariance matrix
	 * to a diagonal matrix with large values to reflect complete uncertainty in the current
	 * pose. If you have any information about the current pose, use reset(const pose_t &,
	 * const pose_t &)
	 */
	void reset();

	/**
	 * Sets the state estimate to the supplied vector and resets the estimate covariance matrix
	 * to a diagonal matrix with large values to reflect complete uncertainty in the current
	 * pose. If you have any information about the current pose, use reset(const pose_t &,
	 * const pose_t &)
	 *
	 * @param pose The pose to which the state estimate will be set.
	 */
	void reset(const pose_t& pose);

	/**
	 * Sets the state estimate to the supplied vector and sets the estimate covariance matrix
	 * to reflect the given uncertainty.
	 *
	 * @param pose The pose to which the state estimate will be set.
	 * @param stdDevs The standard deviation for each element in the pose.
	 */
	void reset(const pose_t& pose, const pose_t& stdDevs);

	/**
	 * Gets the current state estimate.
	 *
	 * @return The state estimate.
	 */
	pose_t getPose() const;

private:
	ExtendedKalmanFilter<numStates, 2, 3, 2, 3> ekf;
	DiffDriveKinematics kinematics;
	double dt;
};

} // namespace filters
