#pragma once

#include "KalmanFilterBase.h"
#include "StateSpaceUtil.h"

#include <array>
#include <functional>

#include <Eigen/Core>
#include <Eigen/LU>

namespace filters {

/**
 * @brief Represent an output for a system. This represents a sensor.
 */
class Output {
public:
	// (state, outputnoise) -> output
	using outputfunc_t =
		std::function<Eigen::VectorXd(const Eigen::VectorXd&, const Eigen::VectorXd&)>;

	const int stateDim;
	const int outputDim;
	const int outputNoiseDim;
	const outputfunc_t outputFunc;
	const statespace::NoiseCovMatX covMat;

	/**
	 * @brief Create an output for a model.
	 *
	 * @param stateDim The dimension of the state vector.
	 * @param outputDim The dimension of the output vector (measurement).
	 * @param outputNoiseDim The dimension of the noise applied to the output.
	 * @param outputFunc The output function that takes a state vector and output noise vector
	 * and produces an output vector.
	 * @param covMat The covariance matrix of the output noise vector. The state dimension must be @p statedim,
	 * the parameter size must @p outputDim and the size must be @p outputnoiseDim
	 */
	Output(int stateDim, int outputDim, int outputNoiseDim, const outputfunc_t& outputFunc,
		   const statespace::NoiseCovMatX& covMat)
		: stateDim(stateDim), outputDim(outputDim), outputNoiseDim(outputNoiseDim),
		  outputFunc(outputFunc), covMat(covMat) {
		assert(covMat.stateDim == stateDim && covMat.size == outputNoiseDim &&
			   covMat.paramDim == outputDim);
	}

	/**
	 * @brief Calculate the output function jacobian at the given state wrt state.
	 *
	 * If an analytic solution has been provided, the jacobian is calculated
	 * using that. Otherwise, it is numerically approximated using the output function.
	 *
	 * @param state The state to evaluate the jacobian at.
	 * @return Eigen::MatrixXd The outputDim x stateDim jacobian matrix.
	 */
	Eigen::MatrixXd outputFuncJacobianX(const Eigen::VectorXd& state) {
		assert(state.size() == stateDim);
		Eigen::VectorXd outputNoise = Eigen::VectorXd::Zero(outputNoiseDim);
		if (outFuncJacobianX) {
			return this->outFuncJacobianX(state, outputNoise);
		} else {
			auto func = [&](const Eigen::VectorXd& s) { return outputFunc(s, outputNoise); };
			return statespace::numericalJacobian(func, state, outputDim);
		}
	}

	/**
	 * @brief Calculate the output function jacobian at the given state wrt the output noise.
	 *
	 * If an analytic solution has been provided, the jacobian is calculated
	 * using that. Otherwise, it is numerically approximated using the output function.
	 *
	 * @param state The state to evaluate the jacobian at.
	 * @return Eigen::MatrixXd The outputDim x processNoiseDim jacobian matrix.
	 */
	Eigen::MatrixXd outputFuncJacobianV(const Eigen::VectorXd& state) {
		assert(state.size() == stateDim);
		if (outFuncJacobianV) {
			Eigen::VectorXd outputNoise = Eigen::VectorXd::Zero(outputNoiseDim);
			return this->outFuncJacobianV(state, outputNoise);
		} else {
			auto func = [&](const Eigen::VectorXd& outputNoise) {
				return outputFunc(state, outputNoise);
			};
			Eigen::VectorXd v = Eigen::VectorXd::Zero(outputNoiseDim);
			return statespace::numericalJacobian(func, v, outputDim);
		}
	}

	/**
	 * @brief Evaluate the output function at the given state.
	 *
	 * @param state The state vector to evaluate at.
	 * @return Eigen::VectorXd An output vector of dimension outputDim.
	 */
	Eigen::VectorXd getOutput(const Eigen::VectorXd& state) {
		assert(state.size() == stateDim);
		Eigen::VectorXd outputNoise = Eigen::VectorXd::Zero(outputNoiseDim);
		return outputFunc(state, outputNoise);
	}

	/**
	 * @brief Set an analytic solution to dh/dx.
	 *
	 * Behavior is undefined if the supplied functions
	 * do not accept or return vectors/matrices of the correct dimensions.
	 *
	 * @param jacobianV The analytic solution to dh/dx.
	 * The jacobian function takes a state vector and an output
	 * noise vector and returns a outputDim x stateDim jacobian matrix.
	 * 
	 * @warning Behavior is undefined if the supplied functions
	 * do not accept or return vectors/matrices of the correct dimensions.
	 */
	void setOutputFuncJacobianX(
		const std::function<Eigen::MatrixXd(const Eigen::VectorXd&, const Eigen::VectorXd&)>&
			jacobianX) {
		outFuncJacobianX = jacobianX;
	}

	/**
	 * @brief Set an analytic solution to dh/dv.
	 *
	 * Behavior is undefined if the supplied functions
	 * do not accept or return vectors/matrices of the correct dimensions.
	 *
	 * @param jacobianV The analytic solution to dh/dv. The jacobian function takes
	 * a state vector and an output noise vector and returns a
	 * outputDim x outputNoiseDim jacobian matrix.
	 * 
	 * @warning Behavior is undefined if the supplied functions
	 * do not accept or return vectors/matrices of the correct dimensions.
	 */
	void setOutputFuncJacobianV(
		const std::function<Eigen::MatrixXd(const Eigen::VectorXd&, const Eigen::VectorXd&)>&
			jacobianV) {
		outFuncJacobianV = jacobianV;
	}

private:
	std::function<Eigen::MatrixXd(const Eigen::VectorXd&, const Eigen::VectorXd&)>
		outFuncJacobianX;

	std::function<Eigen::MatrixXd(const Eigen::VectorXd&, const Eigen::VectorXd&)>
		outFuncJacobianV;
};

template <int stateDim, int inputDim, int processNoiseDim, int numOutputs>
class MultiSensorEKF : public KalmanFilterBase<stateDim, inputDim> {
public:
	using state_t = statespace::Vectord<stateDim>;
	using input_t = statespace::Vectord<inputDim>;
	using processnoise_t = statespace::Vectord<processNoiseDim>;

	using statefunc_t =
		std::function<state_t(const state_t&, const input_t&, const processnoise_t&)>;

	MultiSensorEKF(
		const statefunc_t& stateFunc,
		const statespace::NoiseCovMat<stateDim, processNoiseDim, inputDim>& processNoise,
		double dt, const std::array<Output, numOutputs>& outputs)
		: stateFunc(stateFunc), Q(processNoise), dt(dt), outputs(outputs) {}

	void predict(const input_t& input) override {
		statespace::Matrixd<stateDim, stateDim> F = stateFuncJacobianX(this->xHat, input);
		statespace::Matrixd<processNoiseDim, processNoiseDim> processNoise =
			Q.get(this->xHat, input);

		statespace::Matrixd<stateDim, processNoiseDim> L =
			stateFuncJacobianW(this->xHat, input);

		this->xHat = stateFunc(this->xHat, input, processnoise_t::Zero());
		this->P = F * this->P * F.transpose() + L * processNoise * L.transpose();
	}

	void correct(int outputIdx, const Eigen::VectorXd& measurement) {
		Output& output = outputs[outputIdx];

		Eigen::MatrixXd H = output.outputFuncJacobianX(this->xHat);
		Eigen::MatrixXd outputNoise = output.covMat.get(this->xHat, measurement);
		Eigen::MatrixXd M = output.outputFuncJacobianV(this->xHat);
		Eigen::MatrixXd S = H * this->P * H.transpose() +
							M * outputNoise * M.transpose(); // residual covariance
		// near-optimal kalman gain
		Eigen::Matrix<double, stateDim, Eigen::Dynamic> K =
			S.transpose().colPivHouseholderQr().solve(H * this->P.transpose()).transpose();
		Eigen::VectorXd y =
			measurement -
			output.outputFunc(this->xHat, Eigen::VectorXd::Zero(
											  output.outputNoiseDim)); // measurement residual

		this->xHat = this->xHat + K * y;
		this->P = (Eigen::Matrix<double, stateDim, stateDim>::Identity() - K * H) * this->P;
	}
	
	template <int outputIdx> void correct(const Eigen::VectorXd& measurement) {
		static_assert(0 <= outputIdx && outputIdx < numOutputs);
		correct(outputIdx, measurement);
	}

	void setStateFuncJacobianX(
		const std::function<statespace::Matrixd<stateDim, stateDim>(
			const state_t&, const input_t&, const processnoise_t&)>& stateFuncJacobianX) {
		this->stateFuncJacobianXSol = stateFuncJacobianX;
	}

	void setStateFuncJacobianW(
		const std::function<statespace::Matrixd<stateDim, processNoiseDim>(
			const state_t&, const input_t&, const processnoise_t&)>& stateFuncJacobianW) {
		this->stateFuncJacobianWSol = stateFuncJacobianW;
	}

	statespace::Matrixd<stateDim, stateDim> stateFuncJacobianX(const state_t& x,
															   const input_t& u) {
		processnoise_t w = processnoise_t::Zero();
		if (stateFuncJacobianXSol) {
			return stateFuncJacobianXSol(x, u, w);
		} else {
			auto func = [&](const state_t& state) { return stateFunc(state, u, w); };
			return statespace::numericalJacobian(func, x, stateDim);
		}
	}

	statespace::Matrixd<stateDim, processNoiseDim> stateFuncJacobianW(const state_t& x,
																	  const input_t& u) {
		processnoise_t w = processnoise_t::Zero();
		if (stateFuncJacobianXSol) {
			return stateFuncJacobianXSol(x, u, w);
		} else {
			auto func = [&](const processnoise_t& noise) { return stateFunc(x, u, noise); };
			return statespace::numericalJacobian(func, w, stateDim);
		}
	}

private:
	statefunc_t stateFunc;
	statespace::NoiseCovMat<stateDim, processNoiseDim, inputDim> Q;
	double dt;
	std::array<Output, numOutputs> outputs;

	std::function<statespace::Matrixd<stateDim, stateDim>(const state_t&, const input_t&,
														  const processnoise_t&)>
		stateFuncJacobianXSol;

	std::function<statespace::Matrixd<stateDim, processNoiseDim>(
		const state_t&, const input_t&, const processnoise_t&)>
		stateFuncJacobianWSol;
};

} // namespace filters
