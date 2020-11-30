#pragma once

#include <Eigen/Core>
#include <Eigen/LU>

#include "KalmanFilterBase.h"
#include "StateSpaceUtil.h"

template <int numStates, int size, int paramSize> class NoiseCovMat
{
public:
	using state_t = Eigen::Matrix<double, numStates, 1>;
	using param_t = Eigen::Matrix<double, paramSize, 1>;

	NoiseCovMat(const Eigen::Matrix<double, size, 1> &stdDevs)
		: NoiseCovMat(StateSpace::createCovarianceMatrix(stdDevs))
	{
	}

	NoiseCovMat(const Eigen::Matrix<double, size, size> &mat)
		: func([mat](const state_t &x, const param_t &param) { return mat; })
	{
	}

	NoiseCovMat(const std::function<Eigen::Matrix<double, size, size>(const state_t &,
																	  const param_t &)> &func)
		: func(func)
	{
	}

	Eigen::Matrix<double, size, size> get(const state_t &x, const param_t &param)
	{
		return func(x, param);
	}

private:
	std::function<Eigen::Matrix<double, size, size>(const state_t &, const param_t &)> func;
};

template <int numStates, int numInputs, int numOutputs, int processNoiseDim,
		  int outputNoiseDim>
class ExtendedKalmanFilter : public KalmanFilterBase<numStates, numInputs, numOutputs>
{
public:
	using state_t = Eigen::Matrix<double, numStates, 1>;
	using output_t = Eigen::Matrix<double, numOutputs, 1>;
	using input_t = Eigen::Matrix<double, numInputs, 1>;
	using statefunc_t = std::function<state_t(
		const state_t &, const input_t &, const Eigen::Matrix<double, processNoiseDim, 1> &)>;
	using outputfunc_t = std::function<output_t(
		const state_t &, const Eigen::Matrix<double, outputNoiseDim, 1> &)>;
	using processnoise_t = Eigen::Matrix<double, processNoiseDim, 1>;
	using outputnoise_t = Eigen::Matrix<double, outputNoiseDim, 1>;

	ExtendedKalmanFilter(
		const statefunc_t &stateFunc, const outputfunc_t &outputFunc,
		const NoiseCovMat<numStates, processNoiseDim, numInputs> &processNoise,
		const NoiseCovMat<numStates, outputNoiseDim, numOutputs> &outputNoise, double dt)
		: stateFunc(stateFunc), outputFunc(outputFunc), Q(processNoise), R(outputNoise), dt(dt)
	{
	}

	void predict(const Eigen::Matrix<double, numInputs, 1> &input) override
	{
		Eigen::Matrix<double, numStates, numStates> F =
			getStateFuncJacobianX(stateFunc, this->xHat, input);
		Eigen::Matrix<double, processNoiseDim, processNoiseDim> processNoise =
			Q.get(this->xHat, input);

		Eigen::Matrix<double, numStates, processNoiseDim> L =
			getStateFuncJacobianW(stateFunc, this->xHat, input);

		this->xHat = stateFunc(this->xHat, input, processnoise_t::Zero());
		this->P = F * this->P * F.transpose() + L * processNoise * L.transpose();
	}

	void correct(const Eigen::Matrix<double, numOutputs, 1> &measurement) override
	{
		Eigen::Matrix<double, numOutputs, numStates> H =
			getOutputFuncJacobianX(outputFunc, this->xHat); // output matrix

		Eigen::Matrix<double, outputNoiseDim, outputNoiseDim> outputNoise =
			R.get(this->xHat, measurement);
		Eigen::Matrix<double, numOutputs, outputNoiseDim> M =
			getOutputFuncJacobianV(outputFunc, this->xHat);
		Eigen::Matrix<double, numOutputs, numOutputs> S =
			H * this->P * H.transpose() +
			M * outputNoise * M.transpose(); // residual covariance
		// near-optimal kalman gain
		Eigen::Matrix<double, numStates, numOutputs> K =
			S.transpose().colPivHouseholderQr().solve(H * this->P.transpose()).transpose();
		output_t y = measurement - outputFunc(this->xHat, outputnoise_t::Zero()); // measurement residual

		this->xHat = this->xHat + K * y;
		this->P = (Eigen::Matrix<double, numStates, numStates>::Identity() - K * H) * this->P;
	}

	// manually set this to provide an analytic solution to the jacobian
	std::function<Eigen::Matrix<double, numStates, numStates>(const state_t &, const input_t &,
															  const processnoise_t &)>
		stateFuncJacobianX;
	std::function<Eigen::Matrix<double, numStates, processNoiseDim>(
		const state_t &, const input_t &, const processnoise_t &)>
		stateFuncJacobianW;
	std::function<Eigen::Matrix<double, numOutputs, numStates>(const state_t &,
															   const outputnoise_t &)>
		outputFuncJacobianX;
	std::function<Eigen::Matrix<double, numOutputs, outputNoiseDim>(const state_t &,
																	const outputnoise_t &)>
		outputFuncJacobianV;

protected:
	statefunc_t stateFunc;
	outputfunc_t outputFunc;
	// process noise and measurement noise covariance matrices
	NoiseCovMat<numStates, processNoiseDim, numInputs> Q;
	NoiseCovMat<numStates, outputNoiseDim, numOutputs> R;
	double dt;
	static constexpr double epsilon = 1e-5;

	Eigen::Matrix<double, numStates, numStates>
	getStateFuncJacobianX(const statefunc_t &f, const state_t &x, const input_t &u)
	{
		processnoise_t w = processnoise_t::Zero();
		if (stateFuncJacobianX)
		{
			return stateFuncJacobianX(x, u, w);
		}
		else
		{
			Eigen::Matrix<double, numStates, numStates> jacobian;
			for (int i = 0; i < numStates; i++)
			{
				state_t delta = state_t::Zero();
				delta[i] = epsilon;
				state_t derivative = (f(x + delta, u, w) - f(x - delta, u, w)) / (2 * epsilon);
				jacobian.col(i) = derivative;
			}
			return jacobian;
		}
	}

	Eigen::Matrix<double, numStates, processNoiseDim>
	getStateFuncJacobianW(const statefunc_t &f, const state_t &x, const input_t &u)
	{
		processnoise_t w = processnoise_t::Zero();
		if (stateFuncJacobianW)
		{
			return stateFuncJacobianW(x, u, w);
		}
		else
		{
			Eigen::Matrix<double, numStates, processNoiseDim> jacobian;
			for (int i = 0; i < processNoiseDim; i++)
			{
				processnoise_t delta = processnoise_t ::Zero();
				delta[i] = epsilon;
				state_t derivative = (f(x, u, w + delta) - f(x, u, w - delta)) / (2 * epsilon);
				jacobian.col(i) = derivative;
			}
			return jacobian;
		}
	}

	Eigen::Matrix<double, numOutputs, numStates> getOutputFuncJacobianX(const outputfunc_t &f,
																		const state_t &x)
	{
		outputnoise_t v = outputnoise_t ::Zero();
		if (outputFuncJacobianX)
		{
			return outputFuncJacobianX(x, v);
		}
		else
		{
			Eigen::Matrix<double, numOutputs, numStates> jacobian;
			for (int i = 0; i < numStates; i++)
			{
				state_t delta = Eigen::Matrix<double, numStates, 1>::Zero();
				delta[i] = epsilon;
				output_t derivative = (f(x + delta, v) - f(x - delta, v)) / (2 * epsilon);
				jacobian.col(i) = derivative;
			}
			return jacobian;
		}
	}

	Eigen::Matrix<double, numOutputs, outputNoiseDim>
	getOutputFuncJacobianV(const outputfunc_t &f, const state_t &x)
	{
		outputnoise_t v = outputnoise_t ::Zero();
		if (outputFuncJacobianV)
		{
			return outputFuncJacobianV(x, v);
		}
		else
		{
			Eigen::Matrix<double, numOutputs, outputNoiseDim> jacobian;
			for (int i = 0; i < outputNoiseDim; i++)
			{
				outputnoise_t delta = outputnoise_t::Zero();
				delta[i] = epsilon;
				output_t derivative = (f(x, v + delta) - f(x, v - delta)) / (2 * epsilon);
				jacobian.col(i) = derivative;
			}
			return jacobian;
		}
	}
};
