#pragma once

#include <Eigen/Core>
#include <Eigen/LU>

#include "KalmanFilterBase.h"
#include "StateSpaceUtil.h"

template <int numStates, int numInputs>
class ExtendedKalmanFilter : public KalmanFilterBase<numStates, numInputs>
{
public:
	using matrix_t = Eigen::Matrix<double, numStates, numStates>;
	using state_t = Eigen::Matrix<double, numStates, 1>;
	using input_t = Eigen::Matrix<double, numInputs, 1>;

	ExtendedKalmanFilter(state_t (*stateFunc)(const state_t &, const input_t &),
						 state_t (*measurementFunc)(const state_t &),
						 matrix_t (*stateFuncJacobian)(const state_t &, const input_t &),
						 matrix_t (*measurementFuncJacobian)(const state_t &),
						 const state_t &stateStdDevs, const state_t &measurementStdDevs)
		: stateFunc(stateFunc), measurementFunc(measurementFunc),
		  stateFuncJacobian(stateFuncJacobian),
		  measurementFuncJacobian(measurementFuncJacobian),
		  Q(StateSpace::template createCovarianceMatrix<numStates>(stateStdDevs)),
		  R(StateSpace::template createCovarianceMatrix<numStates>(measurementStdDevs))
	{
		this->P = matrix_t::Zero();
		this->xHat = state_t::Zero();
	}

	void predict(const Eigen::Matrix<double, numInputs, 1> &input) override
	{
		this->xHat = stateFunc(this->xHat, input);
		matrix_t F = stateFuncJacobian(this->xHat, input);
		this->P = F * this->P * F.transpose() + Q;
	}

	void correct(const Eigen::Matrix<double, numStates, 1> &measurement) override
	{
		state_t y = measurement - measurementFunc(this->xHat); // measurement residual
		matrix_t H = measurementFuncJacobian(this->xHat);	   // output matrix
		matrix_t S = H * this->P * H.transpose() + R;		   // residual covariance
		// near-optimal kalman gain
		matrix_t K =
			S.transpose().colPivHouseholderQr().solve(H * this->P.transpose()).transpose();
		this->xHat = this->xHat + K * y;
		this->P = (matrix_t::Identity() - K * H) * this->P;
	}

private:
	state_t (*stateFunc)(const state_t &x, const input_t &u);
	state_t (*measurementFunc)(const state_t &x);
	matrix_t (*stateFuncJacobian)(const state_t &x, const input_t &u);
	matrix_t (*measurementFuncJacobian)(const state_t &x);
	matrix_t Q, R; // process noise and measurement noise covariance matrices
};
