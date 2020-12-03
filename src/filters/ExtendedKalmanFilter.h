#pragma once

#include <Eigen/Core>
#include <Eigen/LU>

#include "KalmanFilterBase.h"
#include "StateSpaceUtil.h"

/**
 * Represents a square noise covariance matrix.
 *
 * @tparam numStates The number of states in this system.
 * @tparam size The size of this matrix.
 * @tparam paramSize The dimension of the vector accepted by get() in addition to a state
 * vector.
 */
template <int numStates, int size, int paramSize> class NoiseCovMat
{
public:
	using state_t = Eigen::Matrix<double, numStates, 1>;
	using param_t = Eigen::Matrix<double, paramSize, 1>;

	/**
	 * Create a time-invariant noise covariance matrix modelling independent noise with the
	 * given standard deviations.
	 *
	 * @param stdDevs The standard deviations of each element.
	 */
	NoiseCovMat(const Eigen::Matrix<double, size, 1> &stdDevs)
		: NoiseCovMat(StateSpace::createCovarianceMatrix(stdDevs))
	{
	}

	/**
	 * Create a time-invariant noise covariance matrix equal to the given matrix.
	 *
	 * @param mat The noise covariance matrix.
	 */
	NoiseCovMat(const Eigen::Matrix<double, size, size> &mat)
		: func([mat](const state_t &x, const param_t &param) { return mat; })
	{
	}

	/**
	 * Create a time-varying noise covariance matrix. At runtime, the matrix will be calculated
	 * when needed using the supplied function.
	 *
	 * @param func The function that supplies the noise covariance matrix,
	 * given the state vector and one additional vector. For process noise, this is the input
	 * vector. For output noise, this is the output vector.
	 */
	NoiseCovMat(const std::function<Eigen::Matrix<double, size, size>(const state_t &,
																	  const param_t &)> &func)
		: func(func)
	{
	}

	/**
	 * Gets the noise covariance matrix, given the current state and additonal parameter.
	 * The matrix may be time-invariant, which case the values of x and param do not matter.
	 *
	 * @param x The current state vector.
	 * @param param The parameter vector, as defined by the use of this matrix.
	 * @return The noise covariance matrix.
	 */
	Eigen::Matrix<double, size, size> get(const state_t &x, const param_t &param)
	{
		return func(x, param);
	}

private:
	std::function<Eigen::Matrix<double, size, size>(const state_t &, const param_t &)> func;
};

/**
 * Implements a discrete-time EKF. This implements the more general system form described here:
 * https://en.wikipedia.org/wiki/Extended_Kalman_filter#Non-additive_noise_formulation_and_equations
 *
 * @tparam numStates The number of states in the system.
 * @tparam numInputs The number of inputs in the system.
 * @tparam numOutputs The number of outputs in the system.
 * @tparam processNoiseDim The dimension of the process noise matrix.
 * In systems with additive process noise, this is the same as numStates.
 * @tparam outputNoiseDim The dimension of the output noise matrix.
 * In systems with additive output noise, this is the same as numOutputs.
 */
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

	/**
	 * Create a new discrete-time EKF.
	 *
	 * @param stateFunc Discrete-time state transition function. x_t+1 = f(x_t, u)
	 * @param outputFunc Output function of the system. Also known as h(x)
	 * @param processNoise The process noise covariance matrix for this system.
	 * @param outputNoise The output noise covariance matrix for this system.
	 * @param dt The time in seconds between updates.
	 */
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
		output_t y = measurement -
					 outputFunc(this->xHat, outputnoise_t::Zero()); // measurement residual

		this->xHat = this->xHat + K * y;
		this->P = (Eigen::Matrix<double, numStates, numStates>::Identity() - K * H) * this->P;
	}

	/**
	 * Set this to provide an analytic solution to df/dx.
	 * If this is null, it will be numerically approximated.
	 */
	std::function<Eigen::Matrix<double, numStates, numStates>(const state_t &, const input_t &,
															  const processnoise_t &)>
		stateFuncJacobianX;
	/**
	 * Set this to provide an analytic solution to df/dw.
	 * If this is null, it will be numerically approximated.
	 */
	std::function<Eigen::Matrix<double, numStates, processNoiseDim>(
		const state_t &, const input_t &, const processnoise_t &)>
		stateFuncJacobianW;
	/**
	 * Set this to provide an analytic solution to dh/dx.
	 * If this is null, it will be numerically approximated.
	 */
	std::function<Eigen::Matrix<double, numOutputs, numStates>(const state_t &,
															   const outputnoise_t &)>
		outputFuncJacobianX;
	/**
	 * Set this to provide an analytic solution to dh/dv.
	 * If this is null, it will be numerically approximated.
	 */
	std::function<Eigen::Matrix<double, numOutputs, outputNoiseDim>(const state_t &,
																	const outputnoise_t &)>
		outputFuncJacobianV;

private:
	statefunc_t stateFunc;
	outputfunc_t outputFunc;
	// process noise and measurement noise covariance matrices
	NoiseCovMat<numStates, processNoiseDim, numInputs> Q;
	NoiseCovMat<numStates, outputNoiseDim, numOutputs> R;
	double dt;
	static constexpr double epsilon = 1e-5;

	Eigen::Matrix<double, numStates, numStates>
	getStateFuncJacobianX(const statefunc_t &f, const state_t &x, const input_t &u) const
	{
		processnoise_t w = processnoise_t::Zero();
		// If we have an analytic solution, use that
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
	getStateFuncJacobianW(const statefunc_t &f, const state_t &x, const input_t &u) const
	{
		processnoise_t w = processnoise_t::Zero();
		// If we have an analytic solution, use that
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
																		const state_t &x) const
	{
		outputnoise_t v = outputnoise_t ::Zero();
		// If we have an analytic solution, use that
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
	getOutputFuncJacobianV(const outputfunc_t &f, const state_t &x) const
	{
		outputnoise_t v = outputnoise_t ::Zero();
		// If we have an analytic solution, use that
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
