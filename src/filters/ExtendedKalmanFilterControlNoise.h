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
		auto stateFunc = this->stateFunc;
		double dt = this->dt;
		Eigen::Matrix<double, numStates, numStates> contA =
			StateSpace::stateFuncJacobianX(stateFunc, this->xHat, input);
		Eigen::Matrix<double, numStates, numInputs> V =
			StateSpace::stateFuncJacobianU(stateFunc, this->xHat, input);
		Eigen::Matrix<double, numStates, numStates> contQ = V * M * V.transpose();

		Eigen::Matrix<double, numStates, numStates> discA, discQ;
		StateSpace::discretizeAQ(contA, contQ, discA, discQ, dt);

		this->xHat = StateSpace::integrateStateFunc(stateFunc, this->xHat, input, dt);
		this->P = discA * this->P * discA.transpose() + discQ;
	}

private:
	Eigen::Matrix<double, numInputs, numInputs> M;
};
