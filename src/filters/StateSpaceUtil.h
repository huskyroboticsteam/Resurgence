#pragma once

#include <Eigen/Core>
#include <Eigen/LU>
#include <unsupported/Eigen/MatrixFunctions>

namespace StateSpace
{

constexpr double epsilon = 1e-5;

template <int size>
Eigen::Matrix<double, size, size>
createCovarianceMatrix(const Eigen::Matrix<double, size, 1> &stdDevs)
{
	Eigen::Matrix<double, size, 1> variances = stdDevs.array().square();
	Eigen::Matrix<double, size, size> mat = variances.asDiagonal();
	return mat;
}

template <int numStates, int numInputs>
void discreteToContinuous(Eigen::Matrix<double, numStates, numStates> &A,
						  Eigen::Matrix<double, numStates, numInputs> &B, double dt)
{
	// Reference paper: https://doi.org/10.1016/0307-904X(80)90177-8
	auto discA = A;
	auto discB = B;
	A = discA.log() / dt;
	B = A * (discA - Eigen::Matrix<double, numStates, numStates>::Identity()).inverse() *
		discB;
}

template <int numStates, int numInputs>
void continuousToDiscrete(Eigen::Matrix<double, numStates, numStates> &A,
						  Eigen::Matrix<double, numStates, numInputs> &B, double dt)
{
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

template <int numStates>
Eigen::Matrix<double, numStates, numStates> discretizeA(const Eigen::Matrix<double, numStates, numStates> &contA, double dt)
{
	// Derived from equations in discreteToContinuous()
	return (contA * dt).exp();
}

// contA is continuous system matrix, contQ is continuous process covariance matrix
template <int numStates>
void discretizeAQ(const Eigen::Matrix<double, numStates, numStates> &contA,
				  const Eigen::Matrix<double, numStates, numStates> &contQ,
				  Eigen::Matrix<double, numStates, numStates> &discA,
				  Eigen::Matrix<double, numStates, numStates> &discQ, double dt)
{
	// zero order hold discretization of the system and state covariance matrices
	// reference:
	// https://en.wikipedia.org/wiki/Discretization#Discretization_of_process_noise
	Eigen::Matrix<double, numStates * 2, numStates * 2> M;
	M.template block<numStates, numStates>(0, 0) = -contA;
	M.template block<numStates, numStates>(0, numStates) = contQ;
	M.template block<numStates, numStates>(numStates, 0).setZero();
	M.template block<numStates, numStates>(numStates, numStates) = contA.transpose();
	M *= dt;

	Eigen::Matrix<double, numStates * 2, numStates * 2> G = M.exp();

	Eigen::Matrix<double, numStates, numStates> AinvQ =
		G.block(0, numStates, numStates, numStates); // this is discA^-1 * Q
	discA = G.block(numStates, numStates, numStates, numStates).transpose();

	Eigen::Matrix<double, numStates, numStates> q = discA * AinvQ; // A * A^-1 * Q = Q
	discQ = (q + q.transpose()) / 2.0; // make Q symmetric again if it became asymmetric
}

// contA is continuous system matrix, contQ is continuous process covariance matrix
template <int numStates>
Eigen::Matrix<double, numStates, numStates>
discretizeQ(const Eigen::Matrix<double, numStates, numStates> &contA,
			const Eigen::Matrix<double, numStates, numStates> &contQ, double dt)
{
	Eigen::Matrix<double, numStates, numStates> discA;
	Eigen::Matrix<double, numStates, numStates> discQ;
	discretizeAQ(contA, contQ, discA, discQ, dt);
	return discQ;
}

template <int numStates>
Eigen::Matrix<double, numStates, numStates>
discretizeQApprox(const Eigen::Matrix<double, numStates, numStates> &contA,
				  const Eigen::Matrix<double, numStates, numStates> &contQ, double dt,
				  int degree = 5)
{
	using matrix_t = Eigen::Matrix<double, numStates, numStates>;
	const matrix_t Atrans = contA.transpose(); // A^T

	matrix_t AinvQ = matrix_t::Zero(); // after iteration, this will equal A^-1 * Q

	double coeff = 1;
	matrix_t AtransN = matrix_t::Identity(); // (A^T)^(n-1)
	matrix_t lastTerm = matrix_t::Zero(); // last term in the series before multiplied by coeff

	// derived from e^(A*t)=I+A+A^2/2...
	// uses the first several terms for the matrix exponential
	for (int i = 1; i <= degree; i++)
	{
		coeff *= dt / i;
		lastTerm = -contA * lastTerm + contQ * AtransN;
		AtransN *= Atrans;
		AinvQ += coeff * lastTerm;
	}

	matrix_t discA = discretizeA(contA, dt);
	matrix_t discQ = discA * AinvQ; // A * A^-1 * Q = Q

	// make Q symmetric again (floating point rounding error may have shifted it)
	return (discQ + discQ.transpose()) / 2.0;
}

// R is measurement covariance matrix
template <int numStates>
Eigen::Matrix<double, numStates, numStates>
discretizeR(const Eigen::Matrix<double, numStates, numStates> &contR, double dt)
{
	return contR / dt;
}

// Solves Discrete-time Algebraic Riccati Equation to calculate asymptotic error covariance
// matrix This can be used to calculate the optimal Kalman gain matrix
template <int numStates>
static Eigen::Matrix<double, numStates, numStates>
DARE(const Eigen::Matrix<double, numStates, numStates> &A0,
	 const Eigen::Matrix<double, numStates, numStates> &Ctrans,
	 const Eigen::Matrix<double, numStates, numStates> &Q,
	 const Eigen::Matrix<double, numStates, numStates> &R)
{
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
	do
	{
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

template <typename T, typename U>
T integrateStateFunc(std::function<T(const T &, const U &)> f, const T &x, const U &u,
					 double dt)
{
	// Reference:
	// https://en.wikipedia.org/wiki/Runge%E2%80%93Kutta_methods#The_Runge%E2%80%93Kutta_method
	double halfDt = dt / 2;
	T k1 = f(x, u);
	T k2 = f(x + halfDt * k1, u);
	T k3 = f(x + halfDt * k2, u);
	T k4 = f(x + dt * k3, u);
	return x + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4);
}

template <int size, int sizeU>
Eigen::Matrix<double, size, size> stateFuncJacobian(
	std::function<Eigen::Matrix<double, size, 1>(const Eigen::Matrix<double, size, 1> &,
												 const Eigen::Matrix<double, sizeU, 1> &)>
		f,
	const Eigen::Matrix<double, size, 1> &x, const Eigen::Matrix<double, sizeU, 1> &u)
{
	Eigen::Matrix<double, size, size> jacobian;
	for (int i = 0; i < size; i++)
	{
		Eigen::Matrix<double, size, 1> delta = Eigen::Matrix<double, size, 1>::Zero();
		delta[i] = epsilon;
		Eigen::Matrix<double, size, 1> derivative =
			(f(x + delta, u) - f(x - delta, u)) / (2 * epsilon);
		jacobian.col(i) = derivative;
	}
	return jacobian;
}

template <int size>
Eigen::Matrix<double, size, size> outputFuncJacobian(
	std::function<Eigen::Matrix<double, size, 1>(const Eigen::Matrix<double, size, 1> &)> f,
	const Eigen::Matrix<double, size, 1> &x)
{
	Eigen::Matrix<double, size, size> jacobian;
	for (int i = 0; i < size; i++)
	{
		Eigen::Matrix<double, size, 1> delta = Eigen::Matrix<double, size, 1>::Zero();
		delta[i] = epsilon;
		Eigen::Matrix<double, size, 1> derivative =
			(f(x + delta) - f(x - delta)) / (2 * epsilon);
		jacobian.col(i) = derivative;
	}
	return jacobian;
}
} // namespace StateSpace
