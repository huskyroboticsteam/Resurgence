#pragma once

#include <Eigen/Core>
#include <Eigen/LU>

#include "KalmanFilterBase.h"
#include "StateSpaceUtil.h"

template <int numStates, int numInputs>
class KalmanFilter : public KalmanFilterBase<numStates, numInputs>
{
public:
	/**
	 * Create a new Kalman Filter from a continuous-time state space model.
	 * The only matrices that change between this and createDisc() are the system matrix and
	 * input matrix. All other parameters would be the same.
	 *
	 * @param systemMat The continuous-time system matrix.
	 * @param inputMat  The continuous-time input matrix.
	 * @param outputMat The output matrix.
	 * @param stateStdDevs The standard deviations for the system model. This implies
	 * inaccuracy in the state space model.
	 * @param measurementStdDevs The standard deviations of the measurement. This implies
	 * inaccuracy in the sensor.
	 * @param dt The time in seconds between each update.
	 * @return A Kalman Filter built for the specified system.
	 */
	static KalmanFilter
	createContinuous(const Eigen::Matrix<double, numStates, numStates> &systemMat,
			   const Eigen::Matrix<double, numStates, numInputs> &inputMat,
			   const Eigen::Matrix<double, numStates, numStates> &outputMat,
			   const Eigen::Matrix<double, numStates, 1> &stateStdDevs,
			   const Eigen::Matrix<double, numStates, 1> &measurementStdDevs, double dt)
	{
		matrix stateCovarianceCont = StateSpace::createCovarianceMatrix(stateStdDevs);
		matrix measurementCovarianceCont =
			StateSpace::createCovarianceMatrix(measurementStdDevs);

		matrix discA = systemMat;
		matrix discB = inputMat;
		StateSpace::continuousToDiscrete(discA, discB, dt);

		matrix stateCovariance = StateSpace::discretizeQ(systemMat, stateCovarianceCont, dt);
		matrix measurementCovariance = StateSpace::discretizeR(measurementCovarianceCont, dt);

		// solve DARE for asymptotic state error covariance matrix
		matrix P = StateSpace::DARE(discA.transpose(), outputMat.transpose(), stateCovariance,
									measurementCovariance);

		matrix S = outputMat * P * outputMat.transpose() + measurementCovariance;
		// This is the Kalman gain matrix, used to weight the GPS data against the model data
		matrix gainMatrix =
			S.transpose().colPivHouseholderQr().solve((outputMat * P.transpose()).transpose());

		return KalmanFilter(discA, discB, outputMat, stateCovariance, measurementCovariance,
							gainMatrix);
	}

	/**
	 * Create a new Kalman Filter from a discrete-time state space model.
	 * The only matrices that change between this and createCont() are the system matrix and
	 * input matrix. All other parameters would be the same.
	 *
	 * @param systemMat The discrete-time system matrix.
	 * @param inputMat  The discrete-time input matrix.
	 * @param outputMat The output matrix.
	 * @param stateStdDevs The standard deviations for the system model. This implies
	 * inaccuracy in the state space model.
	 * @param measurementStdDevs The standard deviations of the measurement. This implies
	 * inaccuracy in the sensor.
	 * @param dt The time in seconds between each update.
	 * @return A Kalman Filter built for the specified system.
	 */
	static KalmanFilter
	createDiscrete(const Eigen::Matrix<double, numStates, numStates> &systemMat,
			   const Eigen::Matrix<double, numStates, numInputs> &inputMat,
			   const Eigen::Matrix<double, numStates, numStates> &outputMat,
			   const Eigen::Matrix<double, numStates, 1> &stateStdDevs,
			   const Eigen::Matrix<double, numStates, 1> &measurementStdDevs, double dt)
	{
		matrix stateCovarianceCont = StateSpace::createCovarianceMatrix(stateStdDevs);
		matrix measurementCovarianceCont =
			StateSpace::createCovarianceMatrix(measurementStdDevs);

		matrix contA = systemMat;
		matrix contB = inputMat;
		StateSpace::discreteToContinuous(contA, contB, dt);

		matrix stateCovariance = StateSpace::discretizeQ(contA, stateCovarianceCont, dt);
		matrix measurementCovariance = StateSpace::discretizeR(measurementCovarianceCont, dt);

		// solve DARE for asymptotic state error covariance matrix
		matrix P = StateSpace::DARE(systemMat.transpose().eval(), outputMat.transpose().eval(),
									stateCovariance, measurementCovariance);

		// The following math is derived from the asymptotic form:
		// https://en.wikipedia.org/wiki/Kalman_filter#Asymptotic_form
		matrix S = outputMat * P * outputMat.transpose() + measurementCovariance;
		// This is the Kalman gain matrix, used to weight the GPS data against the model data
		matrix gainMatrix =
			S.transpose().colPivHouseholderQr().solve(outputMat * P.transpose()).transpose();

		return KalmanFilter(systemMat, inputMat, outputMat, stateCovariance,
							measurementCovariance, gainMatrix);
	}

	void correct(const Eigen::Matrix<double, numStates, 1> &measurement) override
	{
		this->xHat = this->xHat + K * (measurement - C * this->xHat);
		this->P = (matrix::Identity() - K * C) * this->P;
	}

	void predict(const Eigen::Matrix<double, numInputs, 1> &input) override
	{
		this->xHat = A * this->xHat + B * input;
		this->P = A * this->P * A.transpose() + Q;
	}

private:
	using matrix = Eigen::Matrix<double, numStates, numStates>;
	using input_mat = Eigen::Matrix<double, numStates, numInputs>;
	using state = Eigen::Matrix<double, numStates, 1>;

	matrix A;
	input_mat B;
	matrix C;
	matrix Q;
	matrix R;
	matrix K;

	// Assumes all discrete matrices
	KalmanFilter(const matrix &A, const input_mat &B, const matrix &C, const matrix &Q,
				 const matrix &R, matrix K)
		: A(A), B(B), C(C), Q(Q), R(R), K(K)
	{
		this->P = matrix::Zero();
		this->xHat = state::Zero();
	}
};
