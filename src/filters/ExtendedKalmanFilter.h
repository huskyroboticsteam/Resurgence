#pragma once

#include "KalmanFilterBase.h"
#include "StateSpaceUtil.h"

#include <Eigen/Core>
#include <Eigen/LU>

namespace filters {

/**
 * @brief Implements a discrete-time EKF.
 *
 * This implements the more general system form described here:
 * https://en.wikipedia.org/wiki/Extended_Kalman_filter#Non-additive_noise_formulation_and_equations
 *
 * @tparam stateDim The dimension of the state space for this system. This is the number of
 * elements in the state vector.
 * @tparam inputDim The dimension of the input space for this system. This is the number of
 * elements in the input vector.
 * @tparam outputDim The dimension of the output space for this system. This is the number of
 * elements in the output (measurement) vector.
 * @tparam processNoiseDim The dimension of the process noise matrix.
 * In systems with additive process noise, this is the same as stateDim.
 * @tparam outputNoiseDim The dimension of the output noise matrix.
 * In systems with additive output noise, this is the same as outputDim.
 */
template <int stateDim, int inputDim, int outputDim, int processNoiseDim, int outputNoiseDim>
class ExtendedKalmanFilter : public KalmanFilterBase<stateDim, inputDim> {
public:
	using state_t = Eigen::Matrix<double, stateDim, 1>;
	using output_t = Eigen::Matrix<double, outputDim, 1>;
	using input_t = Eigen::Matrix<double, inputDim, 1>;
	using statefunc_t = std::function<state_t(
		const state_t&, const input_t&, const Eigen::Matrix<double, processNoiseDim, 1>&)>;
	using outputfunc_t = std::function<output_t(
		const state_t&, const Eigen::Matrix<double, outputNoiseDim, 1>&)>;
	using processnoise_t = Eigen::Matrix<double, processNoiseDim, 1>;
	using outputnoise_t = Eigen::Matrix<double, outputNoiseDim, 1>;

	/**
	 * @brief Create a new discrete-time EKF.
	 *
	 * @param stateFunc Discrete-time state transition function. x_t+1 = f(x_t, u)
	 * @param outputFunc Output function of the system. Also known as h(x)
	 * @param processNoise The process noise covariance matrix for this system.
	 * @param outputNoise The output noise covariance matrix for this system.
	 * @param dt The time in seconds between updates.
	 */
	ExtendedKalmanFilter(const statefunc_t& stateFunc, const outputfunc_t& outputFunc,
						 const statespace::NoiseCovMat<stateDim, processNoiseDim, inputDim>& processNoise,
						 const statespace::NoiseCovMat<stateDim, outputNoiseDim, outputDim>& outputNoise,
						 double dt)
		: stateFunc(stateFunc), outputFunc(outputFunc), Q(processNoise), R(outputNoise),
		  dt(dt) {}

	void predict(const Eigen::Matrix<double, inputDim, 1>& input) override {
		Eigen::Matrix<double, stateDim, stateDim> F =
			getStateFuncJacobianX(stateFunc, this->xHat, input);
		Eigen::Matrix<double, processNoiseDim, processNoiseDim> processNoise =
			Q.get(this->xHat, input);

		Eigen::Matrix<double, stateDim, processNoiseDim> L =
			getStateFuncJacobianW(stateFunc, this->xHat, input);

		this->xHat = stateFunc(this->xHat, input, processnoise_t::Zero());
		this->P = F * this->P * F.transpose() + L * processNoise * L.transpose();
	}

	/**
	 * @brief Correct the state estimate with measurement data.
	 *
	 * The measurement should be in the same space as the state.
	 *
	 * @param measurement The measurement to use to correct the filter.
	 */
	void correct(const Eigen::Matrix<double, outputDim, 1>& measurement) {
		Eigen::Matrix<double, outputDim, stateDim> H =
			getOutputFuncJacobianX(outputFunc, this->xHat); // output matrix

		Eigen::Matrix<double, outputNoiseDim, outputNoiseDim> outputNoise =
			R.get(this->xHat, measurement);
		Eigen::Matrix<double, outputDim, outputNoiseDim> M =
			getOutputFuncJacobianV(outputFunc, this->xHat);
		Eigen::Matrix<double, outputDim, outputDim> S =
			H * this->P * H.transpose() +
			M * outputNoise * M.transpose(); // residual covariance
		// near-optimal kalman gain
		Eigen::Matrix<double, stateDim, outputDim> K =
			S.transpose().colPivHouseholderQr().solve(H * this->P.transpose()).transpose();
		output_t y = measurement -
					 outputFunc(this->xHat, outputnoise_t::Zero()); // measurement residual

		this->xHat = this->xHat + K * y;
		this->P = (Eigen::Matrix<double, stateDim, stateDim>::Identity() - K * H) * this->P;
	}

	/**
	 * @brief Set this to provide an analytic solution to df/dx.
	 *
	 * If this is null, it will be numerically approximated.
	 */
	std::function<Eigen::Matrix<double, stateDim, stateDim>(const state_t&, const input_t&,
															const processnoise_t&)>
		stateFuncJacobianX;
	/**
	 * @brief Set this to provide an analytic solution to df/dw.
	 *
	 * If this is null, it will be numerically approximated.
	 */
	std::function<Eigen::Matrix<double, stateDim, processNoiseDim>(
		const state_t&, const input_t&, const processnoise_t&)>
		stateFuncJacobianW;
	/**
	 * @brief Set this to provide an analytic solution to dh/dx.
	 *
	 * If this is null, it will be numerically approximated.
	 */
	std::function<Eigen::Matrix<double, outputDim, stateDim>(const state_t&,
															 const outputnoise_t&)>
		outputFuncJacobianX;
	/**
	 * @brief Set this to provide an analytic solution to dh/dv.
	 *
	 * If this is null, it will be numerically approximated.
	 */
	std::function<Eigen::Matrix<double, outputDim, outputNoiseDim>(const state_t&,
																   const outputnoise_t&)>
		outputFuncJacobianV;

private:
	statefunc_t stateFunc;
	outputfunc_t outputFunc;
	// process noise and measurement noise covariance matrices
	statespace::NoiseCovMat<stateDim, processNoiseDim, inputDim> Q;
	statespace::NoiseCovMat<stateDim, outputNoiseDim, outputDim> R;
	double dt;
	static constexpr double epsilon = 1e-5;

	Eigen::Matrix<double, stateDim, stateDim>
	getStateFuncJacobianX(const statefunc_t& f, const state_t& x, const input_t& u) const {
		processnoise_t w = processnoise_t::Zero();
		// If we have an analytic solution, use that
		if (stateFuncJacobianX) {
			return stateFuncJacobianX(x, u, w);
		} else {
			Eigen::Matrix<double, stateDim, stateDim> jacobian;
			for (int i = 0; i < stateDim; i++) {
				state_t delta = state_t::Zero();
				delta[i] = epsilon;
				state_t derivative = (f(x + delta, u, w) - f(x - delta, u, w)) / (2 * epsilon);
				jacobian.col(i) = derivative;
			}
			return jacobian;
		}
	}

	Eigen::Matrix<double, stateDim, processNoiseDim>
	getStateFuncJacobianW(const statefunc_t& f, const state_t& x, const input_t& u) const {
		processnoise_t w = processnoise_t::Zero();
		// If we have an analytic solution, use that
		if (stateFuncJacobianW) {
			return stateFuncJacobianW(x, u, w);
		} else {
			Eigen::Matrix<double, stateDim, processNoiseDim> jacobian;
			for (int i = 0; i < processNoiseDim; i++) {
				processnoise_t delta = processnoise_t::Zero();
				delta[i] = epsilon;
				state_t derivative = (f(x, u, w + delta) - f(x, u, w - delta)) / (2 * epsilon);
				jacobian.col(i) = derivative;
			}
			return jacobian;
		}
	}

	Eigen::Matrix<double, outputDim, stateDim> getOutputFuncJacobianX(const outputfunc_t& f,
																	  const state_t& x) const {
		outputnoise_t v = outputnoise_t::Zero();
		// If we have an analytic solution, use that
		if (outputFuncJacobianX) {
			return outputFuncJacobianX(x, v);
		} else {
			Eigen::Matrix<double, outputDim, stateDim> jacobian;
			for (int i = 0; i < stateDim; i++) {
				state_t delta = Eigen::Matrix<double, stateDim, 1>::Zero();
				delta[i] = epsilon;
				output_t derivative = (f(x + delta, v) - f(x - delta, v)) / (2 * epsilon);
				jacobian.col(i) = derivative;
			}
			return jacobian;
		}
	}

	Eigen::Matrix<double, outputDim, outputNoiseDim>
	getOutputFuncJacobianV(const outputfunc_t& f, const state_t& x) const {
		outputnoise_t v = outputnoise_t::Zero();
		// If we have an analytic solution, use that
		if (outputFuncJacobianV) {
			return outputFuncJacobianV(x, v);
		} else {
			Eigen::Matrix<double, outputDim, outputNoiseDim> jacobian;
			for (int i = 0; i < outputNoiseDim; i++) {
				outputnoise_t delta = outputnoise_t::Zero();
				delta[i] = epsilon;
				output_t derivative = (f(x, v + delta) - f(x, v - delta)) / (2 * epsilon);
				jacobian.col(i) = derivative;
			}
			return jacobian;
		}
	}
};

} // namespace filters
