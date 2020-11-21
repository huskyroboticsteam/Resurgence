#pragma once

#include <Eigen/Core>

template <int numStates, int numInputs, int numOutputs>
class KalmanFilterBase {
public:
	/**
	 * Correct the state estimate with measurement data.
	 * The measurement should be in the same space as the state.
	 *
	 * @param measurement The measurement to use to correct the filter.
	 */
	virtual void correct(const Eigen::Matrix<double, numOutputs, 1> &measurement) = 0;

	/**
	 * Use the model to predict the next system state, given the current inputs.
	 *
	 * @param input The current inputs to the system.
	 */
	virtual void predict(const Eigen::Matrix<double, numInputs, 1> &input) = 0;

	/**
	 * Sets the state estimate to the supplied vector and resets the estimate covariance
	 * matrix.
	 *
	 * @param state The state to which the state estimate will be set.
	 */
	void reset(const Eigen::Matrix<double, numStates, 1> &state)
	{
		xHat = state;
		P = Eigen::Matrix<double, numStates, numStates>::Zero();
	}

	/**
	 * Sets the state estimate to the zero vector and resets the estimate covariance matrix.
	 */
	void reset()
	{
		reset(Eigen::Matrix<double, numStates, 1>::Zero());
	}

	/**
	 * Gets the current state estimate.
	 *
	 * @return The state estimate.
	 */
	Eigen::Matrix<double, numStates, 1> getState() const
	{
		return xHat;
	}

	/**
	 * Get the current estimate covariance matrix.
	 * This is an indication of the uncertainty of the pose estimate.
	 *
	 * @return The estimate covariance matrix, AKA the P matrix.
	 */
	Eigen::Matrix<double, numStates, numStates> getEstimateCovarianceMat() const
	{
		return P;
	}

protected:
	Eigen::Matrix<double, numStates, numStates> P;
	Eigen::Matrix<double, numStates, 1> xHat;
};
