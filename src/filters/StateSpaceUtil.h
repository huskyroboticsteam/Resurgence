#pragma once

#include <Eigen/Core>
#include <Eigen/LU>
#include <unsupported/Eigen/MatrixFunctions>

/**
 * @namespace filters::statespace
 * @brief Collection of utility methods for use with state space applications.
 */
namespace filters::statespace {

constexpr double epsilon = 1e-5;

/**
 * @brief Create a covariance matrix modelling independent variables with the given standard
 * deviations.
 *
 * @tparam size The dimension of the noise.
 * @param stdDevs The standard deviation of each element.
 * @return A size x size covariance matrix.
 */
template <int size>
Eigen::Matrix<double, size, size>
createCovarianceMatrix(const Eigen::Matrix<double, size, 1>& stdDevs) {
	Eigen::Matrix<double, size, 1> variances = stdDevs.array().square();
	Eigen::Matrix<double, size, size> mat = variances.asDiagonal();
	return mat;
}

/**
 * @brief Convert a discrete time system matrix to continuous time.
 *
 * @tparam numStates The number of states in the system.
 * @param A The discrete time system matrix.
 * @param dt The time in seconds between updates.
 * @return An equivalent continuous time system matrix.
 */
template <int numStates>
Eigen::Matrix<double, numStates, numStates>
discreteToContinuous(const Eigen::Matrix<double, numStates, numStates>& A, double dt) {
	return A.log() / dt;
}

/**
 * @brief Convert discrete time system and input matrices to continuous time.
 *
 * @tparam numStates The number of states in the system.
 * @param A A reference to the discrete time system matrix. It will be replaced with the
 * continuous time equivalent.
 * @param B A reference to the discrete time input matrix. It will be replaced with the
 * continuous time equivalent.
 * @param dt The time in seconds between updates.
 */
template <int numStates, int numInputs>
void discreteToContinuous(Eigen::Matrix<double, numStates, numStates>& A,
						  Eigen::Matrix<double, numStates, numInputs>& B, double dt) {
	// Reference paper: https://doi.org/10.1016/0307-904X(80)90177-8
	auto discA = A;
	auto discB = B;
	A = discA.log() / dt;
	B = A * (discA - Eigen::Matrix<double, numStates, numStates>::Identity()).inverse() *
		discB;
}

/**
 * @brief Convert continuous time system and input matrices to discrete time.
 *
 * @tparam numStates The number of states in the system.
 * @param A A reference to the continuous time system matrix. It will be replaced with the
 * discrete time equivalent.
 * @param B A reference to the continuous time input matrix. It will be replaced with the
 * discrete time equivalent.
 * @param dt The time in seconds between updates.
 */
template <int numStates, int numInputs>
void continuousToDiscrete(Eigen::Matrix<double, numStates, numStates>& A,
						  Eigen::Matrix<double, numStates, numInputs>& B, double dt) {
	// zero order hold discretization for system and input matrices
	// reference:
	// https://en.wikipedia.org/wiki/Discretization#Discretization_of_linear_state_space_models
	Eigen::Matrix<double, numStates + numInputs, numStates + numInputs> M;
	M.setZero();
	M.template block<numStates, numStates>(0, 0) = A;
	M.template block<numStates, numInputs>(0, numStates) = B;
	M *= dt;

	Eigen::Matrix<double, numStates + numInputs, numStates + numInputs> exp = M.exp();
	A = exp.template block<numStates, numStates>(0, 0);
	B = exp.template block<numStates, numInputs>(0, numStates);
}

/**
 * @brief Convert a continuous time system matrix to discrete time.
 *
 * @tparam numStates The number of states in the system.
 * @param contA The continuous time system matrix.
 * @param dt The time in seconds between updates.
 * @return An equivalent discrete time system matrix.
 */
template <int numStates>
Eigen::Matrix<double, numStates, numStates>
discretizeA(const Eigen::Matrix<double, numStates, numStates>& contA, double dt) {
	// Derived from equations in discreteToContinuous()
	return (contA * dt).exp();
}

/**
 * @brief Convert a continuous time system matrix and additive process noise matrix to discrete time.
 *
 * @tparam numStates The number of states in the system.
 * @param contA A reference to the continuous time system matrix.
 * @param contQ A reference to the continuous time additive process noise matrix.
 * @param discA A reference to a matrix where the discrete time system matrix will be stored.
 * @param discQ A reference to a matrix where the discrete process noise covariance matrix will
 * be stored.
 * @param dt The time in seconds between updates.
 */
template <int numStates>
void discretizeAQ(const Eigen::Matrix<double, numStates, numStates>& contA,
				  const Eigen::Matrix<double, numStates, numStates>& contQ,
				  Eigen::Matrix<double, numStates, numStates>& discA,
				  Eigen::Matrix<double, numStates, numStates>& discQ, double dt) {
	// zero order hold discretization of the system and state covariance matrices
	// reference:
	// https://en.wikipedia.org/wiki/Discretization#Discretization_of_process_noise
	Eigen::Matrix<double, numStates * 2, numStates * 2> M;
	M.template block<numStates, numStates>(0, 0) = -contA;
	M.template block<numStates, numStates>(0, numStates) = contQ;
	M.template block<numStates, numStates>(numStates, 0).setZero();
	M.template block<numStates, numStates>(numStates, numStates) = contA.transpose();
	M *= dt;

	Eigen::Matrix<double, numStates * 2, numStates* 2> G = M.exp();

	Eigen::Matrix<double, numStates, numStates> AinvQ =
		G.block(0, numStates, numStates, numStates); // this is discA^-1 * Q
	discA = G.block(numStates, numStates, numStates, numStates).transpose();

	Eigen::Matrix<double, numStates, numStates> q = discA * AinvQ; // A * A^-1 * Q = Q
	discQ = (q + q.transpose()) / 2.0; // make Q symmetric again if it became asymmetric
}

/**
 * @brief Discretizes a continuous time additive process noise covariance matrix.
 *
 * @tparam numStates The number of states in the system.
 * @param contA The continuous time system matrix.
 * @param contQ The continuous time process noise covariance matrix.
 * @param dt The time in second between updates.
 * @return The discrete time process noise covariance matrix.
 */
template <int numStates>
Eigen::Matrix<double, numStates, numStates>
discretizeQ(const Eigen::Matrix<double, numStates, numStates>& contA,
			const Eigen::Matrix<double, numStates, numStates>& contQ, double dt) {
	Eigen::Matrix<double, numStates, numStates> discA;
	Eigen::Matrix<double, numStates, numStates> discQ;
	discretizeAQ(contA, contQ, discA, discQ, dt);
	return discQ;
}

/**
 * @brief Discretizes a continuous time additive output noise covariance matrix.
 *
 * @tparam numStates The number of states in the system.
 * @param contR The continuous time output noise covariance matrix.
 * @param dt The time in seconds between updates.
 * @return The discrete time output noise covariance matrix.
 */
template <int numStates>
Eigen::Matrix<double, numStates, numStates>
discretizeR(const Eigen::Matrix<double, numStates, numStates>& contR, double dt) {
	return contR / dt;
}

/**
 * @brief Solves the Discrete-time Algebraic Riccati Equation.
 *
 * This can be used to calculate the asymptotic
 * estimate error covariance matrix. (P_inf) This can be used to calculate the optimal
 * Kalman gain matrix.
 *
 * @tparam numStates The number of states in the system.
 * @param A0 The discrete time system matrix.
 * @param Ctrans The transpose of the discrete time output matrix.
 * @param Q The discrete time additive process noise covariance matrix.
 * @param R The discrete time additive output noise covariance matrix.
 * @return The asymptotic estimate error covariance matrix.
 */
template <int numStates>
static Eigen::Matrix<double, numStates, numStates>
DARE(const Eigen::Matrix<double, numStates, numStates>& A0,
	 const Eigen::Matrix<double, numStates, numStates>& Ctrans,
	 const Eigen::Matrix<double, numStates, numStates>& Q,
	 const Eigen::Matrix<double, numStates, numStates>& R) {
	// reference:
	// https://scicomp.stackexchange.com/questions/30757/discrete-time-algebraic-riccati-equation-dare-solver-in-c
	Eigen::Matrix<double, numStates, numStates> A = A0;
	Eigen::Matrix<double, numStates, numStates> G = Ctrans * R.inverse() * Ctrans.transpose();
	Eigen::Matrix<double, numStates, numStates> H = Q;
	const Eigen::Matrix<double, numStates, numStates> I =
		Eigen::Matrix<double, numStates, numStates>::Identity();

	Eigen::Matrix<double, numStates, numStates> lastA;
	Eigen::Matrix<double, numStates, numStates> lastG;
	Eigen::Matrix<double, numStates, numStates> lastH;
	// converges quadratically
	do {
		lastA = A;
		lastG = G;
		lastH = H;

		const Eigen::Matrix<double, numStates, numStates> AIGH =
			lastA * (I + lastG * lastH).inverse();
		A = lastA * AIGH * lastA;
		G = lastG + lastA * AIGH * lastG * lastA.transpose();
		H = lastH + lastA.transpose() * lastH * (I + lastG * lastH).transpose() * lastA;
	} while ((H - lastH).norm() / H.norm() >= 1e-4);

	return H;
}
} // namespace StateSpace
