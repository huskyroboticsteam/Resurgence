#pragma once

#include "StateSpaceUtil.h"

#include <Eigen/Core>

namespace filters {

template <int stateDim, int inputDim, int outputDim> class KalmanFilterBase {
public:
	KalmanFilterBase()
		: P(Eigen::Matrix<double, stateDim, stateDim>::Identity() * 1e5),
		  xHat(Eigen::Matrix<double, stateDim, 1>::Zero()) {}

	/**
	 * Correct the state estimate with measurement data.
	 * The measurement should be in the same space as the state.
	 *
	 * @param measurement The measurement to use to correct the filter.
	 */
	virtual void correct(const Eigen::Matrix<double, outputDim, 1>& measurement) = 0;

	/**
	 * Use the model to predict the next system state, given the current inputs.
	 *
	 * @param input The current inputs to the system.
	 */
	virtual void predict(const Eigen::Matrix<double, inputDim, 1>& input) = 0;

	/**
	 * Sets the state estimate to the zero vector and resets the estimate covariance matrix to
	 * a diagonal matrix with large values.
	 */
	void reset() {
		reset(Eigen::Matrix<double, stateDim, 1>::Zero());
	}

	/**
	 * Sets the state estimate to the supplied vector and resets the estimate covariance
	 * matrix to a diagonal matrix with large values.
	 *
	 * @param state The state to which the state estimate will be set.
	 */
	void reset(const Eigen::Matrix<double, stateDim, 1>& state) {
		Eigen::Matrix<double, stateDim, stateDim> newP =
			Eigen::Matrix<double, stateDim, stateDim>::Identity() * 1e5;
		reset(state, newP);
	}

	/**
	 * Sets the state estimate to the supplied vector with the given uncertainty.
	 *
	 * @param state The state to set the vector to.
	 * @param stdDevs The standard deviation of the measurement of each element in the state
	 * vector. This should not be the zero vector. If you want to represent high certainty, use
	 * very small values instead.
	 */
	void reset(const Eigen::Matrix<double, stateDim, 1>& state,
			   const Eigen::Matrix<double, stateDim, 1>& stdDevs) {
		reset(state, statespace::createCovarianceMatrix(stdDevs));
	}

	/**
	 * Sets the state estimate to the supplied vector and the estimate covariance matrix to the
	 * supplied matrix.
	 *
	 * @param state The state to set the vector to.
	 * @param estCovMat The matrix to set the estimate covariance matrix to. This should not be
	 * the zero matrix. If you want to represent high certainty, use a diagonal matrix with
	 * very small values.
	 */
	void reset(const Eigen::Matrix<double, stateDim, 1>& state,
			   const Eigen::Matrix<double, stateDim, stateDim>& estCovMat) {
		xHat = state;
		P = estCovMat;
	}

	/**
	 * Gets the current state estimate.
	 *
	 * @return The state estimate.
	 */
	Eigen::Matrix<double, stateDim, 1> getState() const {
		return xHat;
	}

	/**
	 * Get the current estimate covariance matrix.
	 * This is an indication of the uncertainty of the pose estimate.
	 *
	 * @return The estimate covariance matrix, AKA the P matrix.
	 */
	Eigen::Matrix<double, stateDim, stateDim> getEstimateCovarianceMat() const {
		return P;
	}

protected:
	Eigen::Matrix<double, stateDim, stateDim> P;
	Eigen::Matrix<double, stateDim, 1> xHat;
};

} // namespace filters
