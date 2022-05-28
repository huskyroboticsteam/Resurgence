#pragma once

#include "../filters/StateSpaceUtil.h"

#include <functional>
#include <optional>

#include <Eigen/Core>

template <int outputDim, int inputDim> class JacobianVelController {
private:
	template <int dim> using Vectord = filters::statespace::Vectord<dim>;
	template <int rows, int cols> using Matrixd = filters::statespace::Matrixd<rows, cols>;

public:
	JacobianVelController(
		const std::function<Vectord<outputDim>(const Vectord<inputDim>&)>& kinematicsFunc,
		const std::function<Matrixd<outputDim, inputDim>(const Vectord<inputDim>&)>&
			jacobianFunc,
		double dt) {}

	Vectord<inputDim> getCommand(const Vectord<inputDim>& currPos) const {
		double unused;
		return getCommand(currPos, unused);
	}

	Vectord<inputDim> getCommand(const Vectord<inputDim>& currPos, double& cosineSim) const {
		if (!controllerState) {
			return currPos;
		}
		Vectord<outputDim> currPosOutput = kinematicsFunc(currPos);
		state_t state = controllerState.value();
		Vectord<outputDim> lastTarget = state.lastTarget.value_or(currPosOutput);
		Vectord<outputDim> currTarget = lastTarget + dt * state.targetVel;
		state.lastTarget = currTarget;
		Vectord<outputDim> outputPosDiff = currTarget - currPosOutput;
		Matrixd<outputDim, inputDim> jacobian = jacobianFunc(currPos);
		Vectord<inputDim> inputPosDiff = jacobian.colPivHouseholderQR().solve(outputPosDiff);

		Vectord<outputDim> trueOutputPosDiff = jacobian * inputPosDiff;
		cosineSim = outputPosDiff.dot(trueOutputPosDiff) /
					(outputPosDiff.norm() * trueOutputPosDiff.norm());

		return currPos + inputPosDiff;
	}

	void setTarget(const Vectord<outputDim>& targetVel) {
		state_t state = {.targetVel = targetVel, .lastTarget = {}};
		this->controllerState = state;
	}

	bool hasTarget() {
		return controllerState.has_value();
	}

	void reset() {
		controllerState.reset();
	}

private:
	struct state_t {
		Vectord<outputDim> targetVel;
		std::optional<Vectord<outputDim>> lastTarget;
	};

	std::function<Vectord<outputDim>(const Vectord<inputDim>&)> kinematicsFunc;
	std::function<Matrixd<outputDim, inputDim>(const Vectord<inputDim>&)> jacobianFunc;
	double dt;
	std::optional<state_t> controllerState;
};
