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

	ExtendedKalmanFilter(
		const std::function<state_t(const state_t &, const input_t &)> &stateFunc,
		const std::function<state_t(const state_t &)> &measurementFunc,
		const state_t &stateStdDevs, const state_t &measurementStdDevs, double dt)
		: stateFunc(stateFunc), measurementFunc(measurementFunc),
		  Q(StateSpace::template createCovarianceMatrix<numStates>(stateStdDevs)),
		  R(StateSpace::template createCovarianceMatrix<numStates>(measurementStdDevs)), dt(dt)
	{
		this->P = matrix_t::Zero();
		this->xHat = state_t::Zero();
	}

	void predict(const Eigen::Matrix<double, numInputs, 1> &input) override
	{
		matrix_t contA = StateSpace::stateFuncJacobian(stateFunc, this->xHat, input);
		matrix_t discA, discQ;
		StateSpace::discretizeAQ(contA, Q, discA, discQ, dt);

		this->xHat = StateSpace::integrateStateFunc(stateFunc, this->xHat, input, dt);
		this->P = discA * this->P * discA.transpose() + discQ;
	}

	void correct(const Eigen::Matrix<double, numStates, 1> &measurement) override
	{
		matrix_t C = StateSpace::outputFuncJacobian(measurementFunc, this->xHat); // output matrix

		matrix_t discR = StateSpace::discretizeR(R, dt);
		matrix_t S = C * this->P * C.transpose() + discR; // residual covariance
		// near-optimal kalman gain
		matrix_t K =
			S.transpose().colPivHouseholderQr().solve(C * this->P.transpose()).transpose();
		state_t y = measurement - measurementFunc(this->xHat); // measurement residual

		this->xHat = this->xHat + K * y;
		this->P = (matrix_t::Identity() - K * C) * this->P;
	}

private:
	std::function<state_t(const state_t &, const input_t &)> stateFunc;
	std::function<state_t(const state_t &)> measurementFunc;
	matrix_t Q, R; // process noise and measurement noise covariance matrices
	double dt;
};
