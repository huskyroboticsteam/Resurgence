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

Eigen::MatrixXd
numericalJacobian(const std::function<Eigen::VectorXd(const Eigen::VectorXd&)>& func,
				  const Eigen::VectorXd& x, int outputDim);

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
 * @brief Convert a continuous time system matrix and additive process noise matrix to discrete
 * time.
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
 * @brief Solve a discrete-time Algebraic Riccati equation.
 *
 * The matrix names used are the same convention as the linked wikipedia article.
 * Note that these matrices do not have specific names since DARE is simply a type
 * of equation.
 *
 * @tparam numStates The dimension of the matrices.
 * @param A The A matrix.
 * @param B The B matrix.
 * @param R The R matrix.
 * @param Q The Q matrix.
 * @param tolerance The algorithm is stopped when the relative error of the calculation is at
 * most this value.
 * @param maxIter If nonnegative, the algorithm will stop after these many iterations if it has
 * not converged by then.
 * @return Eigen::Matrix<double, numStates, numStates> The approximate solution to the DARE.
 *
 * @see https://en.wikipedia.org/wiki/Algebraic_Riccati_equation#Solution
 */
template <int numStates>
static Eigen::Matrix<double, numStates, numStates>
DARE(const Eigen::Matrix<double, numStates, numStates>& A,
	 const Eigen::Matrix<double, numStates, numStates>& B,
	 const Eigen::Matrix<double, numStates, numStates>& Q,
	 const Eigen::Matrix<double, numStates, numStates>& R, double tolerance = 1e-4,
	 int maxIter = -1) {
	using mat_t = Eigen::Matrix<double, numStates, numStates>;
	// see:
	// https://scicomp.stackexchange.com/questions/30757/discrete-time-algebraic-riccati-equation-dare-solver-in-c
	// to conform to the wikipedia article conventions, we replace H with P

	mat_t currA = A;
	mat_t currG = B * R.inverse() * B.transpose();
	mat_t currP = Q;

	mat_t I = mat_t::Identity();

	double error;

	for (int i = 0; maxIter < 0 || i < maxIter; i++) {
		mat_t lastA = currA;
		mat_t lastG = currG;
		mat_t lastP = currP;

		mat_t AIGHinv = lastA * (I + lastG * lastP).inverse();

		currA = AIGHinv * lastA;
		currG = lastG + AIGHinv * lastG * lastA.transpose();
		currP = lastP + lastA.transpose() * lastP * (I + lastG * lastP).inverse() * lastA;

		double error = (currP - lastP).norm() / currP.norm();
		if (error > tolerance){
			break;
		}
	}

	return currP;
}

/**
 * @brief Represents a square noise covariance matrix.
 *
 * Returning the zero matrix can sometimes be
 * dangerous depending on your model, as it may cause numerical instability or incorrect
 * computations.
 *
 * @tparam stateDim The dimension of the state space for this system. This is the number of
 * elements in the state vector.
 * @tparam size The size of this matrix.
 * @tparam paramSize The dimension of the vector accepted by get() in addition to a state
 * vector.
 */
template <int stateDim, int size, int paramSize> class NoiseCovMat {
public:
	using state_t = Eigen::Matrix<double, stateDim, 1>;
	using param_t = Eigen::Matrix<double, paramSize, 1>;

	static_assert(stateDim > 0 && size > 0 && paramSize > 0, "Positive sizes are required!");

	/**
	 * @brief Create a time-invariant noise covariance matrix modelling independent noise with the
	 * given standard deviations.
	 *
	 * @param stdDevs The standard deviations of each element.
	 */
	explicit NoiseCovMat(const Eigen::Matrix<double, size, 1>& stdDevs)
		: NoiseCovMat(createCovarianceMatrix(stdDevs)) {}

	/**
	 * @brief Create a time-invariant noise covariance matrix equal to the given matrix.
	 *
	 * @param mat The noise covariance matrix.
	 */
	NoiseCovMat(const Eigen::Matrix<double, size, size>& mat)
		: func([mat](const state_t& x, const param_t& param) { return mat; }) {}

	/**
	 * @brief Create a time-varying noise covariance matrix.
	 *
	 * At runtime, the matrix will be calculated
	 * when needed using the supplied function.
	 *
	 * @param func The function that supplies the noise covariance matrix,
	 * given the state vector and one additional vector. For process noise, this is usually the
	 * input vector. For output noise, this is usually the output vector.
	 */
	NoiseCovMat(const std::function<Eigen::Matrix<double, size, size>(const state_t&,
																	  const param_t&)>& func)
		: func(func) {}

	/**
	 * @brief Gets the noise covariance matrix, given the current state and additonal parameter.
	 *
	 * The matrix may be time-invariant, which case the values of x and param do not matter.
	 *
	 * @param x The current state vector.
	 * @param param The parameter vector, as defined by the use of this matrix. For process
	 * noise, this is usually the input vector. For output noise this is usually the output
	 * vector.
	 * @return The noise covariance matrix.
	 */
	Eigen::Matrix<double, size, size> get(const state_t& x, const param_t& param) {
		return func(x, param);
	}

private:
	std::function<Eigen::Matrix<double, size, size>(const state_t&, const param_t&)> func;
};

/**
 * @brief Represents a square noise covariance matrix.
 *
 * Returning the zero matrix can sometimes be
 * dangerous depending on your model, as it may cause numerical instability or incorrect
 * computations.
 *
 * This is a specialization of NoiseCovMat, where dimensions are unknown at compile time.
 */
template <>
class NoiseCovMat<-1, -1, -1> {
public:
	const int stateDim, size, paramDim;

	/**
	 * @brief Create a time-invariant noise covariance matrix modelling independent noise with
	 * the given standard deviations.
	 *
	 * @param stdDevs The standard deviations of each element.
	 */
	NoiseCovMat(const Eigen::VectorXd& stdDevs, int stateDim, int paramSize);

	/**
	 * @brief Create a time-invariant noise covariance matrix equal to the given matrix.
	 *
	 * @param mat The noise covariance matrix.
	 */
	NoiseCovMat(const Eigen::MatrixXd& mat, int stateDim, int paramSize);

	/**
	 * @brief Create a time-varying noise covariance matrix.
	 *
	 * At runtime, the matrix will be calculated
	 * when needed using the supplied function.
	 *
	 * @param func The function that supplies the noise covariance matrix,
	 * given the state vector and one additional vector. For process noise, this is usually the
	 * input vector. For output noise, this is usually the output vector.
	 */
	NoiseCovMat(const std::function<Eigen::MatrixXd(const Eigen::VectorXd&,
													const Eigen::VectorXd&)>& func,
				int stateDim, int size, int paramSize);

	/**
	 * @brief Gets the noise covariance matrix, given the current state and additonal
	 * parameter.
	 *
	 * The matrix may be time-invariant, which case the values of x and param do not matter.
	 *
	 * @param x The current state vector.
	 * @param param The parameter vector, as defined by the use of this matrix. For process
	 * noise, this is usually the input vector. For output noise this is usually the output
	 * vector.
	 * @return The noise covariance matrix.
	 */
	Eigen::MatrixXd get(const Eigen::VectorXd& x, const Eigen::VectorXd& param) const;

private:
	std::function<Eigen::MatrixXd(const Eigen::VectorXd&, const Eigen::VectorXd&)> func;
};

using NoiseCovMatX = NoiseCovMat<-1, -1, -1>;

} // namespace filters::statespace
