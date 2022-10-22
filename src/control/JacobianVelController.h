#pragma once

#include "../filters/StateSpaceUtil.h"

#include <functional>
#include <optional>

#include <Eigen/Core>

/**
 * @brief This class controls the velocity of a multidimensional mechanism
 * by commanding its position using the jacobian of the kinematics.
 *
 * @tparam outputDim The dimension of the output vector of the kinematic function.
 * @tparam inputDim The dimension of the input vector of the kinematic function.
 * @see https://en.wikipedia.org/wiki/Jacobian_matrix_and_determinant
 */
template <int outputDim, int inputDim> class JacobianVelController {
private:
	template <int dim> using Vectord = filters::statespace::Vectord<dim>;
	template <int rows, int cols> using Matrixd = filters::statespace::Matrixd<rows, cols>;

public:
	/**
	 * @brief Construct a new controller.
	 *
	 * @param kinematicsFunc The kinematics of the mechanism being controlled. This is
	 * a function that takes in some input vector (ex: joint angles of an arm) and outputs
	 * another vector (ex: the 3d pose of the hand)
	 * @param jacobianFunc The jacobian of the kinematic function.
	 * @param dt The time delta of this controller.
	 * When executing a command, getCommand() should be called at this rate.
	 */
	JacobianVelController(
		const std::function<Vectord<outputDim>(const Vectord<inputDim>&)>& kinematicsFunc,
		const std::function<Matrixd<outputDim, inputDim>(const Vectord<inputDim>&)>&
			jacobianFunc,
		double dt) : kinematicsFunc(kinematicsFunc), jacobianFunc(jacobianFunc), dt(dt) {}

	/**
	 * @brief Get the current target position.
	 *
	 * @param currPos The current position of the mechanism, in the kinematic input space.
	 * @return Vectord<inputDim> The target position of the mechanism, in the kinematic input space.
	 */
	Vectord<inputDim> getCommand(const Vectord<inputDim>& currPos) const {
		double unused;
		return getCommand(currPos, unused);
	}

	/**
	 * @brief Get the current target position.
	 *
	 * @param currPos The current position of the mechanism, in the kinematic input sequence.
	 * @param[out] cosineSim Output parameter, gives the cosine similarity of the delta to the returned target and the delta to the commanded target.
	 * This similarity may be low if the kinematics of the mechanism do not allow it to move in the commanded direction.
	 * @return Vectord<inputDim> The target position of the mechanism, in the kinematic input space.
	 * Returns the current position if no target is set.
	 */
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

	/**
	 * @brief Set the target velocity.
	 *
	 * @param targetVel The target velocity, in the output space of the kinematic function.
	 */
	void setTarget(const Vectord<outputDim>& targetVel) {
		state_t state = {.targetVel = targetVel, .lastTarget = {}};
		this->controllerState = state;
	}

	/**
	 * @brief Check if a target has been set.
	 *
	 * @return bool True iff a target has been set.
	 */
	bool hasTarget() {
		return controllerState.has_value();
	}

	/**
	 * @brief Reset the controller, unsetting the current command.
	 */
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
