#pragma once

#include "ExtendedKalmanFilter.h"
#include "StateSpaceUtil.h"

#include "Eigen/Core"

template <int numStates, int numInputs, int numOutputs>
class ExtendedKalmanFilterControlNoise
	: public ExtendedKalmanFilter<numStates, numInputs, numOutputs>
{
public:
	using state_t = Eigen::Matrix<double, numStates, 1>;
	using output_t = Eigen::Matrix<double, numOutputs, 1>;
	using input_t = Eigen::Matrix<double, numInputs, 1>;

	ExtendedKalmanFilterControlNoise(
		const std::function<state_t(const state_t &, const input_t &)> &stateFunc,
		const std::function<output_t(const state_t &)> &measurementFunc,
		const input_t &inputStdDevs, const output_t &measurementStdDevs, double dt)
		: ExtendedKalmanFilter<numStates, numInputs, numOutputs>(
			  stateFunc, measurementFunc, state_t::Zero(), measurementStdDevs, dt),
		  M(StateSpace::createCovarianceMatrix(inputStdDevs))
	{
	}

	void predict(const Eigen::Matrix<double, numInputs, 1> &input) override
	{
		// TODO: This definitely shouldn't be hard coded. Make it configurable
		// In fact, this entire class can be folded into the regular EKF class with some work
		// TODO: Redefine statefunc and outputfunc to take w and v
		Eigen::Vector2d stds = {30*0.04*sqrt(abs(input(0))), 30*0.04*sqrt(abs(input(1)))};
		Eigen::Matrix<double, 2, 2> M = StateSpace::createCovarianceMatrix(stds);

		auto stateFunc = this->stateFunc;
		Eigen::Matrix<double, numStates, numStates> F =
			this->getStateFuncJacobianX(stateFunc, this->xHat, input);
		Eigen::Matrix<double, numStates, numInputs> V =
			getStateFuncJacobianU(stateFunc, this->xHat, input);
		Eigen::Matrix<double, numStates, numStates> Q = V * M * V.transpose();

		this->xHat = stateFunc(this->xHat, input);
		this->P = F * this->P * F.transpose() + Q;
	}

	std::function<Eigen::Matrix<double, numStates, numInputs>(const state_t &,
															  const input_t &)>
		stateFuncJacobianU;

private:
	Eigen::Matrix<double, numInputs, numInputs> M;

	Eigen::Matrix<double, numStates, numInputs>
	getStateFuncJacobianU(std::function<state_t(const state_t &, const input_t &)> f,
						  const state_t &x, const input_t &u)
	{
		if (stateFuncJacobianU)
		{
			return stateFuncJacobianU(x, u);
		}
		else
		{
			return StateSpace::stateFuncJacobianU(f, x, u);
		}
	}
};
