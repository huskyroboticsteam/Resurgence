#pragma once

#include <Eigen/Core>
#include <Eigen/LU>

#include "KalmanFilterBase.h"
#include "StateSpaceUtil.h"

template <int numStates, int numInputs, int numOutputs>
class ExtendedKalmanFilter : public KalmanFilterBase<numStates, numInputs, numOutputs>
{
public:
	using state_t = Eigen::Matrix<double, numStates, 1>;
	using output_t = Eigen::Matrix<double, numOutputs, 1>;
	using input_t = Eigen::Matrix<double, numInputs, 1>;

	ExtendedKalmanFilter(
		const std::function<state_t(const state_t &, const input_t &)> &stateFunc,
		const std::function<output_t(const state_t &)> &measurementFunc,
		const state_t &stateStdDevs, const output_t &measurementStdDevs, double dt)
		: stateFunc(stateFunc), measurementFunc(measurementFunc),
		  Q(StateSpace::createCovarianceMatrix<numStates>(stateStdDevs)),
		  R(StateSpace::createCovarianceMatrix<numOutputs>(measurementStdDevs)), dt(dt)
	{
		this->P = Eigen::Matrix<double, numStates, numStates>::Zero();
		this->xHat = state_t::Zero();
	}

	void predict(const Eigen::Matrix<double, numInputs, 1> &input) override
	{
		Eigen::Matrix<double, numStates, numStates> contA =
			getStateFuncJacobianX(stateFunc, this->xHat, input);
		Eigen::Matrix<double, numStates, numStates> discA, discQ;
		StateSpace::discretizeAQ(contA, Q, discA, discQ, dt);

		this->xHat = StateSpace::integrateStateFunc(stateFunc, this->xHat, input, dt);
		this->P = discA * this->P * discA.transpose() + discQ;
	}

	void correct(const Eigen::Matrix<double, numOutputs, 1> &measurement) override
	{
		Eigen::Matrix<double, numOutputs, numStates> C =
			getOutputFuncJacobian(measurementFunc, this->xHat); // output matrix

		Eigen::Matrix<double, numOutputs, numOutputs> discR = StateSpace::discretizeR(R, dt);
		Eigen::Matrix<double, numOutputs, numOutputs> S =
			C * this->P * C.transpose() + discR; // residual covariance
		// near-optimal kalman gain
		Eigen::Matrix<double, numStates, numOutputs> K =
			S.transpose().colPivHouseholderQr().solve(C * this->P.transpose()).transpose();
		output_t y = measurement - measurementFunc(this->xHat); // measurement residual

		this->xHat = this->xHat + K * y;
		this->P = (Eigen::Matrix<double, numStates, numStates>::Identity() - K * C) * this->P;
	}

	// manually set this to provide an analytic solution to the jacobian
	std::function<Eigen::Matrix<double, numStates, numStates>(const state_t &,
															  const input_t &)>
		stateFuncJacobianX;
	std::function<Eigen::Matrix<double, numOutputs, numStates>(const state_t &)>
		outputFuncJacobian;

protected:
	std::function<state_t(const state_t &, const input_t &)> stateFunc;
	std::function<output_t(const state_t &)> measurementFunc;
	// process noise and measurement noise covariance matrices
	Eigen::Matrix<double, numStates, numStates> Q;
	Eigen::Matrix<double, numOutputs, numOutputs> R;
	double dt;

	Eigen::Matrix<double, numStates, numStates>
	getStateFuncJacobianX(std::function<state_t(const state_t &, const input_t &)> f,
						  const state_t &x, const input_t &u)
	{
		if (stateFuncJacobianX)
		{
			return stateFuncJacobianX(x, u);
		}
		else
		{
			return StateSpace::stateFuncJacobianX(f, x, u);
		}
	}

	Eigen::Matrix<double, numOutputs, numStates>
	getOutputFuncJacobian(std::function<output_t(const state_t &)> f,
						  const state_t &x)
	{
		if (outputFuncJacobian)
		{
			return outputFuncJacobian(x);
		}
		else
		{
			return StateSpace::outputFuncJacobian(f, x);
		}
	}
};
