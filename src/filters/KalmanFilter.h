//
// Created by abdes on 10/31/2020.
//

#ifndef ROVER_KALMANFILTER_H
#define ROVER_KALMANFILTER_H

#include <Eigen/Core>
#include <Eigen/LU>
#include <unsupported/Eigen/MatrixFunctions>

template <int numStates, int numInputs> class KalmanFilter
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
	createCont(const Eigen::Matrix<double, numStates, numStates> &systemMat,
			   const Eigen::Matrix<double, numStates, numInputs> &inputMat,
			   const Eigen::Matrix<double, numStates, numStates> &outputMat,
			   const Eigen::Matrix<double, numStates, 1> &stateStdDevs,
			   const Eigen::Matrix<double, numStates, 1> &measurementStdDevs, double dt)
	{
		matrix stateCovarianceCont = createCovarianceMatrix(stateStdDevs);
		matrix measurementCovarianceCont = createCovarianceMatrix(measurementStdDevs);

		matrix discA = systemMat;
		matrix discB = inputMat;
		continuousToDiscrete(discA, discB, dt);

		matrix stateCovariance = discretizeQ(systemMat, stateCovarianceCont, dt);
		matrix measurementCovariance = discretizeR(measurementCovarianceCont, dt);

		// solve DARE for asymptotic state error covariance matrix
		matrix P = DARE(discA.transpose(), outputMat.transpose(), stateCovariance,
						measurementStdDevs);

		matrix S = outputMat * P * outputMat.transpose() + measurementStdDevs;
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
	createDisc(const Eigen::Matrix<double, numStates, numStates> &systemMat,
			   const Eigen::Matrix<double, numStates, numInputs> &inputMat,
			   const Eigen::Matrix<double, numStates, numStates> &outputMat,
			   const Eigen::Matrix<double, numStates, 1> &stateStdDevs,
			   const Eigen::Matrix<double, numStates, 1> &measurementStdDevs, double dt)
	{
		matrix stateCovarianceCont = createCovarianceMatrix(stateStdDevs);
		matrix measurementCovarianceCont = createCovarianceMatrix(measurementStdDevs);

		matrix contA = systemMat;
		matrix contB = inputMat;
		discreteToContinuous(contA, contB, dt);

		matrix stateCovariance = discretizeQ(contA, stateCovarianceCont, dt);
		matrix measurementCovariance = discretizeR(measurementCovarianceCont, dt);

		// solve DARE for asymptotic state error covariance matrix
		matrix P = DARE(systemMat.transpose(), outputMat.transpose(), stateCovariance,
						measurementStdDevs);

		matrix S = outputMat * P * outputMat.transpose() + measurementStdDevs;
		// This is the Kalman gain matrix, used to weight the GPS data against the model data
		matrix gainMatrix =
			S.transpose().colPivHouseholderQr().solve((outputMat * P.transpose()).transpose());

		return KalmanFilter(systemMat, inputMat, outputMat, stateCovariance,
							measurementCovariance, gainMatrix);
	}

	/**
	 * Correct the state estimate with measurement data.
	 * The measurement should be in the same space as the state.
	 *
	 * @param measurement The measurement to use to correct the filter.
	 */
	void correct(const Eigen::Matrix<double, numStates, 1> &measurement)
	{
		xHat = xHat + K * (measurement - C * xHat);
		P = (matrix::Identity() - K * C) * P;
	}

	/**
	 * Use the model to predict the next system state, given the current inputs.
	 *
	 * @param input The current inputs to the system.
	 */
	void predict(const Eigen::Matrix<double, numInputs, 1> &input)
	{
		xHat = A * xHat + B * input;
		P = A * P * A.transpose() + Q;
	}

	/**
	 * Sets the state estimate to the supplied vector and resets the estimate covariance
	 * matrix.
	 *
	 * @param state The state to which the state estimate will be set.
	 */
	void reset(const Eigen::Matrix<double, numStates, 1> &state)
	{
		xHat = state;
		P = matrix::Zero();
	}

	/**
	 * Sets the state estimate to the zero vector and resets the estimate covariance matrix.
	 */
	void reset()
	{
		reset(state::Zero());
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
	matrix P;
	state xHat;

	// Assumes all discrete matrices
	KalmanFilter(const matrix &A, const input_mat &B, const matrix &C, const matrix &Q,
				 const matrix &R, matrix K)
		: A(A), B(B), C(C), Q(Q), R(R), K(K), P(matrix::Identity()), xHat(state::Zero())
	{
	}

	template <int size>
	static Eigen::Matrix<double, size, size>
	createCovarianceMatrix(const Eigen::Matrix<double, size, 1> &stdDevs)
	{
		Eigen::Matrix<double, size, 1> variances = stdDevs.array().square();
		Eigen::Matrix<double, size, size> mat = variances.asDiagonal();
		return mat;
	}

	static void discreteToContinuous(matrix &A, input_mat &B, double dt)
	{
		// Reference paper: https://doi.org/10.1016/0307-904X(80)90177-8
		matrix G = A;
		input_mat H = B;
		A = G.log() / dt;
		B = A * (G - Eigen::Matrix3d::Identity()).inverse() * H;
	}

	static void continuousToDiscrete(matrix &A, input_mat &B, double dt)
	{
		Eigen::Matrix<double, numStates + numInputs, numStates + numInputs> M;
		M.setZero();
		M.template block<numStates, numStates>(0, 0) = A;
		M.template block<numStates, numInputs>(0, numStates) = B;
		M *= dt;

		Eigen::Matrix<double, numStates + numInputs, numStates + numInputs> exp = M.exp();
		A = exp.template block<numStates, numStates>(0, 0);
		B = exp.template block<numStates, numInputs>(0, numStates);
	}

	// contA is continuous system matrix, contQ is continuous process covariance matrix
	static matrix discretizeQ(const matrix &contA, const matrix &contQ, double dt)
	{
		// implements zero order hold discretization of the system and state covariance
		// matrices reference:
		// https://en.wikipedia.org/wiki/Discretization#Discretization_of_process_noise
		Eigen::Matrix<double, numStates * 2, numStates * 2> M;
		M.template block<numStates, numStates>(0, 0) = -contA;
		M.template block<numStates, numStates>(0, numStates) = contQ;
		M.template block<numStates, numStates>(numStates, 0).setZero();
		M.template block<numStates, numStates>(numStates, numStates) = contA.transpose();

		Eigen::Matrix<double, numStates * 2, numStates * 2> G = (M * dt).exp();

		matrix AinvQ = G.block(0, numStates, numStates, numStates); // this is discA^-1 * Q
		matrix discA = G.block(numStates, numStates, numStates, numStates).transpose();

		matrix q = discA * AinvQ;	   // A * A^-1 * Q = Q
		q = (q + q.transpose()) / 2.0; // make Q symmetric again if it became asymmetric

		return q;
	}

	// R is measurement covariance matrix
	static matrix discretizeR(const matrix &contR, double dt)
	{
		return contR / dt;
	}

	// Solves Discrete-time Algebraic Riccati Equation to calculate asymptotic error covariance
	// matrix This can be used to calculate the optimal Kalman gain matrix
	static matrix DARE(const matrix &A0, const matrix &Ctrans, const matrix &Q,
					   const matrix &R)
	{
		// reference:
		// https://scicomp.stackexchange.com/questions/30757/discrete-time-algebraic-riccati-equation-dare-solver-in-c
		matrix A = A0;
		matrix G = Ctrans * R.inverse() * Ctrans.transpose();
		matrix H = Q;
		const matrix I = matrix::Identity();

		matrix lastA;
		matrix lastG;
		matrix lastH;
		// converges quadratically
		do
		{
			lastA = A;
			lastG = G;
			lastH = H;

			const matrix AIGH = lastA * (I + lastG * lastH).inverse();
			A = lastA * AIGH * lastA;
			G = lastG + lastA * AIGH * lastG * lastA.transpose();
			H = lastH + lastA.transpose() * lastH * (I + lastG * lastH).transpose() * lastA;
		} while ((H - lastH).norm() / H.norm() >= 0.01);

		return H;
	}
};

#endif // ROVER_KALMANFILTER_H
