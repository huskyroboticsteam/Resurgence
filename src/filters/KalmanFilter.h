#pragma once

#include <Eigen/Core>
#include <Eigen/LU>

#include "KalmanFilterBase.h"
#include "StateSpaceUtil.h"

template <int numStates, int numInputs, int numOutputs>
class KalmanFilter : public KalmanFilterBase<numStates, numInputs, numOutputs>
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
					 const Eigen::Matrix<double, numOutputs, numStates> &outputMat,
					 const Eigen::Matrix<double, numStates, 1> &stateStdDevs,
					 const Eigen::Matrix<double, numOutputs, 1> &measurementStdDevs, double dt)
	{
		Eigen::Matrix<double, numStates, numStates> stateCovarianceCont = StateSpace::createCovarianceMatrix(stateStdDevs);
		Eigen::Matrix<double, numOutputs, numOutputs> measurementCovarianceCont =
			StateSpace::createCovarianceMatrix(measurementStdDevs);

		Eigen::Matrix<double, numStates, numStates> discA = systemMat;
		Eigen::Matrix<double, numStates, numInputs> discB = inputMat;
		StateSpace::continuousToDiscrete(discA, discB, dt);

		Eigen::Matrix<double, numStates, numStates> stateCovariance = StateSpace::discretizeQ(systemMat, stateCovarianceCont, dt);
		Eigen::Matrix<double, numOutputs, numOutputs> measurementCovariance = StateSpace::discretizeR(measurementCovarianceCont, dt);

		// solve DARE for asymptotic state error covariance matrix
		Eigen::Matrix<double, numStates, numStates> P = StateSpace::DARE(discA.transpose(), outputMat.transpose(), stateCovariance,
									measurementCovariance);

		Eigen::Matrix<double, numOutputs, numOutputs> S = outputMat * P * outputMat.transpose() + measurementCovariance;
		// This is the Kalman gain matrix, used to weight the GPS data against the model data
		Eigen::Matrix<double, numStates, numOutputs> gainMatrix =
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
				   const Eigen::Matrix<double, numOutputs, numStates> &outputMat,
				   const Eigen::Matrix<double, numStates, 1> &stateStdDevs,
				   const Eigen::Matrix<double, numOutputs, 1> &measurementStdDevs, double dt)
	{
		Eigen::Matrix<double, numStates, numStates> stateCovarianceCont = StateSpace::createCovarianceMatrix(stateStdDevs);
		Eigen::Matrix<double, numOutputs, numOutputs> measurementCovarianceCont =
			StateSpace::createCovarianceMatrix(measurementStdDevs);

		Eigen::Matrix<double, numStates, numStates> contA = systemMat;
		Eigen::Matrix<double, numStates, numInputs> contB = inputMat;
		StateSpace::discreteToContinuous(contA, contB, dt);

		Eigen::Matrix<double, numStates, numStates> stateCovariance = StateSpace::discretizeQ(contA, stateCovarianceCont, dt);
		Eigen::Matrix<double, numOutputs, numOutputs> measurementCovariance = StateSpace::discretizeR(measurementCovarianceCont, dt);

		// solve DARE for asymptotic state error covariance matrix
		Eigen::Matrix<double, numStates, numStates> P = StateSpace::DARE(systemMat.transpose().eval(), outputMat.transpose().eval(),
									stateCovariance, measurementCovariance);

		// The following math is derived from the asymptotic form:
		// https://en.wikipedia.org/wiki/Kalman_filter#Asymptotic_form
		Eigen::Matrix<double, numOutputs, numOutputs> S = outputMat * P * outputMat.transpose() + measurementCovariance;
		// This is the Kalman gain matrix, used to weight the GPS data against the model data
		Eigen::Matrix<double, numStates, numOutputs> gainMatrix =
			S.transpose().colPivHouseholderQr().solve(outputMat * P.transpose()).transpose();

		return KalmanFilter(systemMat, inputMat, outputMat, stateCovariance,
							measurementCovariance, gainMatrix);
	}

	void correct(const Eigen::Matrix<double, numStates, 1> &measurement) override
	{
		this->xHat = this->xHat + K * (measurement - C * this->xHat);
		this->P = (Eigen::Matrix<double, numStates, numStates>::Identity() - K * C) * this->P;
	}

	void predict(const Eigen::Matrix<double, numInputs, 1> &input) override
	{
		this->xHat = A * this->xHat + B * input;
		this->P = A * this->P * A.transpose() + Q;
	}

private:
	Eigen::Matrix<double, numStates, numStates> A;
	Eigen::Matrix<double, numStates, numInputs> B;
	Eigen::Matrix<double, numInputs, numStates> C;
	Eigen::Matrix<double, numStates, numStates> Q;
	Eigen::Matrix<double, numOutputs, numOutputs> R;
	Eigen::Matrix<double, numStates, numOutputs> K;

	// Assumes all discrete matrices
	KalmanFilter(const Eigen::Matrix<double, numStates, numStates> &A,
				 const Eigen::Matrix<double, numStates, numInputs> &B,
				 const Eigen::Matrix<double, numInputs, numStates> &C,
				 const Eigen::Matrix<double, numStates, numStates> &Q,
				 const Eigen::Matrix<double, numOutputs, numOutputs> &R,
				 const Eigen::Matrix<double, numStates, numOutputs> &K)
		: A(A), B(B), C(C), Q(Q), R(R), K(K)
	{
		this->P = Eigen::Matrix<double, numStates, numStates> ::Zero();
		this->xHat = Eigen::Matrix<double, numStates, 1> ::Zero();
	}
};
