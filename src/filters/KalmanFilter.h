#pragma once

#include "KalmanFilterBase.h"
#include "StateSpaceUtil.h"

#include <Eigen/Core>
#include <Eigen/LU>

template <int stateDim, int inputDim, int outputDim>
class KalmanFilter : public KalmanFilterBase<stateDim, inputDim, outputDim> {
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
	createContinuous(const Eigen::Matrix<double, stateDim, stateDim>& systemMat,
					 const Eigen::Matrix<double, stateDim, inputDim>& inputMat,
					 const Eigen::Matrix<double, outputDim, stateDim>& outputMat,
					 const Eigen::Matrix<double, stateDim, 1>& stateStdDevs,
					 const Eigen::Matrix<double, outputDim, 1>& measurementStdDevs,
					 double dt) {
		Eigen::Matrix<double, stateDim, stateDim> stateCovarianceCont =
			StateSpace::createCovarianceMatrix(stateStdDevs);
		Eigen::Matrix<double, outputDim, outputDim> measurementCovarianceCont =
			StateSpace::createCovarianceMatrix(measurementStdDevs);

		Eigen::Matrix<double, stateDim, stateDim> discA = systemMat;
		Eigen::Matrix<double, stateDim, inputDim> discB = inputMat;
		StateSpace::continuousToDiscrete(discA, discB, dt);

		Eigen::Matrix<double, stateDim, stateDim> stateCovariance =
			StateSpace::discretizeQ(systemMat, stateCovarianceCont, dt);
		Eigen::Matrix<double, outputDim, outputDim> measurementCovariance =
			StateSpace::discretizeR(measurementCovarianceCont, dt);

		// solve DARE for asymptotic state error covariance matrix
		Eigen::Matrix<double, stateDim, stateDim> P = StateSpace::DARE(
			discA.transpose(), outputMat.transpose(), stateCovariance, measurementCovariance);

		Eigen::Matrix<double, outputDim, outputDim> S =
			outputMat * P * outputMat.transpose() + measurementCovariance;
		// This is the Kalman gain matrix, used to weight the GPS data against the model data
		Eigen::Matrix<double, stateDim, outputDim> gainMatrix =
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
	createDiscrete(const Eigen::Matrix<double, stateDim, stateDim>& systemMat,
				   const Eigen::Matrix<double, stateDim, inputDim>& inputMat,
				   const Eigen::Matrix<double, outputDim, stateDim>& outputMat,
				   const Eigen::Matrix<double, stateDim, 1>& stateStdDevs,
				   const Eigen::Matrix<double, outputDim, 1>& measurementStdDevs, double dt) {
		Eigen::Matrix<double, stateDim, stateDim> stateCovarianceCont =
			StateSpace::createCovarianceMatrix(stateStdDevs);
		Eigen::Matrix<double, outputDim, outputDim> measurementCovarianceCont =
			StateSpace::createCovarianceMatrix(measurementStdDevs);

		Eigen::Matrix<double, stateDim, stateDim> contA = systemMat;
		Eigen::Matrix<double, stateDim, inputDim> contB = inputMat;
		StateSpace::discreteToContinuous(contA, contB, dt);

		Eigen::Matrix<double, stateDim, stateDim> stateCovariance =
			StateSpace::discretizeQ(contA, stateCovarianceCont, dt);
		Eigen::Matrix<double, outputDim, outputDim> measurementCovariance =
			StateSpace::discretizeR(measurementCovarianceCont, dt);

		// solve DARE for asymptotic state error covariance matrix
		Eigen::Matrix<double, stateDim, stateDim> P =
			StateSpace::DARE(systemMat.transpose().eval(), outputMat.transpose().eval(),
							 stateCovariance, measurementCovariance);

		// The following math is derived from the asymptotic form:
		// https://en.wikipedia.org/wiki/Kalman_filter#Asymptotic_form
		Eigen::Matrix<double, outputDim, outputDim> S =
			outputMat * P * outputMat.transpose() + measurementCovariance;
		// This is the Kalman gain matrix, used to weight the GPS data against the model data
		Eigen::Matrix<double, stateDim, outputDim> gainMatrix =
			S.transpose().colPivHouseholderQr().solve(outputMat * P.transpose()).transpose();

		return KalmanFilter(systemMat, inputMat, outputMat, stateCovariance,
							measurementCovariance, gainMatrix);
	}

	void correct(const Eigen::Matrix<double, stateDim, 1>& measurement) override {
		this->xHat = this->xHat + K * (measurement - C * this->xHat);
		this->P = (Eigen::Matrix<double, stateDim, stateDim>::Identity() - K * C) * this->P;
	}

	void predict(const Eigen::Matrix<double, inputDim, 1>& input) override {
		this->xHat = A * this->xHat + B * input;
		this->P = A * this->P * A.transpose() + Q;
	}

private:
	Eigen::Matrix<double, stateDim, stateDim> A;
	Eigen::Matrix<double, stateDim, inputDim> B;
	Eigen::Matrix<double, inputDim, stateDim> C;
	Eigen::Matrix<double, stateDim, stateDim> Q;
	Eigen::Matrix<double, outputDim, outputDim> R;
	Eigen::Matrix<double, stateDim, outputDim> K;

	// Assumes all discrete matrices
	KalmanFilter(const Eigen::Matrix<double, stateDim, stateDim>& A,
				 const Eigen::Matrix<double, stateDim, inputDim>& B,
				 const Eigen::Matrix<double, inputDim, stateDim>& C,
				 const Eigen::Matrix<double, stateDim, stateDim>& Q,
				 const Eigen::Matrix<double, outputDim, outputDim>& R,
				 const Eigen::Matrix<double, stateDim, outputDim>& K)
		: A(A), B(B), C(C), Q(Q), R(R), K(K) {
	}
};
