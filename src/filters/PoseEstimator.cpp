#include "PoseEstimator.h"

#include <Eigen/LU>
#include <unsupported/Eigen/MatrixFunctions>

#include "../simulator/utils.h"

typedef Eigen::Matrix3d matrix;
typedef Eigen::Vector3d vector;

matrix createCovarianceMatrix(const vector &stdDevs)
{
	vector variances = stdDevs.array().square();
	matrix mat = variances.asDiagonal();
	return mat;
}

void discreteToContinuous(matrix &A, matrix &B, double dt)
{
	matrix G = A;
	matrix H = B;
	A = G.log() / dt;
	B = A * (G - Eigen::Matrix3d::Identity()).inverse() * H;
}

// contQ is covariance matrix
matrix discretizeQ(const matrix &contA, const matrix &contQ, double dt)
{
	Eigen::Matrix<double, 6, 6> M;
	M.block<3, 3>(0, 0) = -contA;
	M.block<3, 3>(0, 3) = contQ;
	M.block<3, 3>(3, 0).setZero();
	M.block<3, 3>(3, 3) = contA.transpose();

	Eigen::Matrix<double, 6, 6> phi = (M * dt).exp();

	// Phi12 = phi[0:States,        States:2*States]
	// Phi22 = phi[States:2*States, States:2*States]
	Eigen::Matrix3d phi12 = phi.block(0, 3, 3, 3);
	Eigen::Matrix<double, 3, 3> phi22 = phi.block(3, 3, 3, 3);

	matrix discA = phi22.transpose();

	matrix q = discA * phi12;
	q = (q + q.transpose()) / 2.0;

	return q;
}

// R is measurement covariance matrix
matrix discretizeR(const matrix &contR, double dt)
{
	return contR / dt;
}

// Solves Discrete-time Algebraic Riccati Equation
// reference:
// https://scicomp.stackexchange.com/questions/30757/discrete-time-algebraic-riccati-equation-dare-solver-in-c
matrix DARE(const matrix &A0, const matrix &B, const matrix &Q, const matrix &R)
{
	matrix A = A0;
	matrix G = B * R.inverse() * B.transpose();
	matrix H = Q;
	const matrix I = matrix::Identity();

	matrix lastA, lastG, lastH;
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

PoseEstimator::PoseEstimator(const Eigen::Vector3d &stateStdDevs,
							 const Eigen::Vector3d &measurementStdDevs, double dt)
	: dt(dt), systemMat(matrix::Identity()), inputMat(matrix::Identity()),
	  outputMat(matrix::Identity()), estimateCovariance(matrix::Zero())
{
	matrix stateCovarianceCont = createCovarianceMatrix(stateStdDevs);
	matrix measurementCovarianceCont = createCovarianceMatrix(measurementStdDevs);

	matrix contA = systemMat;
	matrix contB = inputMat;
	discreteToContinuous(contA, contB, dt);

	stateCovariance = discretizeQ(contA, stateCovarianceCont, dt);
	measurementCovariance = discretizeR(measurementCovarianceCont, dt);

	// solve DARE for state error covariance matrix
	matrix P = DARE(systemMat.transpose(), outputMat.transpose(), stateCovariance, measurementCovariance);

	matrix S = outputMat * P * outputMat.transpose() + measurementCovariance;
	// This is the Kalman gain matrix, used to weight the GPS data against the model data
	gainMatrix = S.transpose().colPivHouseholderQr().solve((outputMat * P.transpose()).transpose());
}

vector getPoseDiff(const vector &pose, double dt, double thetaVel, double xVel)
{
	double dx = xVel * dt;
	double dTheta = thetaVel * dt;

	vector updated = toPose(toTransformRotateFirst(dx, 0, dTheta) * toTransform(pose), 0);
	return updated - pose;
}

void PoseEstimator::predict(double thetaVel, double xVel)
{
	vector u = getPoseDiff(xHat, dt, thetaVel, xVel);
	xHat = systemMat * xHat + inputMat * u;
	estimateCovariance = systemMat * estimateCovariance * systemMat.transpose() + stateCovariance;
}

void PoseEstimator::correct(const Eigen::Vector3d &measurement)
{
	vector u(0, 0, 0);
	xHat = xHat + gainMatrix * (measurement - outputMat * xHat);
	estimateCovariance = (matrix::Identity() - gainMatrix * outputMat) * estimateCovariance;
}

void PoseEstimator::reset(const Eigen::Vector3d &pose)
{
	xHat = pose;
	estimateCovariance = matrix::Zero();
}
