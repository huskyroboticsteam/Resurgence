#pragma once

#include "../Util.h"
#include "../filters/StateSpaceUtil.h"
#include "../navtypes.h"
#include "../utils/math.h"
#include "../world_interface/data.h"

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
public:
	/**
	 * @brief Construct a new controller.
	 *
	 * @param kinematicsFunc The kinematics of the mechanism being controlled. This is
	 * a function that takes in some input vector (ex: joint angles of an arm) and outputs
	 * another vector (ex: the 3d pose of the hand)
	 * @param jacobianFunc The jacobian of the kinematic function.
	 */
	JacobianVelController(
		const std::function<navtypes::Vectord<outputDim>(const navtypes::Vectord<inputDim>&)>&
			kinematicsFunc,
		const std::function<navtypes::Matrixd<outputDim, inputDim>(
			const navtypes::Vectord<inputDim>&)>& jacobianFunc)
		: kinematicsFunc(kinematicsFunc),
		  jacobianFunc(jacobianFunc ? jacobianFunc
									: std::bind(util::numericalJacobian, kinematicsFunc,
												std::placeholders::_1, outputDim)) {
		assert(this->kinematicsFunc);
		assert(this->jacobianFunc);
	}

	/**
	 * @brief Get the current target position.
	 *
	 * @param currTime The current timestamp.
	 * @param currPos The current position of the mechanism, in the kinematic input space.
	 * @return navtypes::Vectord<inputDim> The target position of the mechanism, in the
	 * kinematic input space.
	 */
	navtypes::Vectord<inputDim> getCommand(robot::types::datatime_t currTime,
										   const navtypes::Vectord<inputDim>& currPos) {
		double unused;
		return getCommand(currTime, currPos, unused);
	}

	/**
	 * @brief Get the current target position.
	 *
	 * @param currTime The current timestamp.
	 * @param currPos The current position of the mechanism, in the kinematic input sequence.
	 * @param[out] cosineSim Output parameter, gives the cosine similarity of the delta to the
	 * returned target and the delta to the commanded target. This similarity may be low if the
	 * kinematics of the mechanism do not allow it to move in the commanded direction.
	 * @return navtypes::Vectord<inputDim> The target position of the mechanism, in the
	 * kinematic input space. Returns the current position if no target is set.
	 */
	navtypes::Vectord<inputDim> getCommand(robot::types::datatime_t currTime,
										   const navtypes::Vectord<inputDim>& currPos,
										   double& cosineSim) {
		if (!controllerState) {
			return currPos;
		}

		// The jacobian of the kinematics function represents a linearization of the kinematics
		// around the current position. We then use this approximation to find the position
		// delta in the input space that yields the required position delta in the output space
		// This is similar to JacobianController, but the command position is calculated
		// from a velocity instead of a profile.

		navtypes::Vectord<outputDim> currPosOutput = kinematicsFunc(currPos);
		state_t state = controllerState.value();
		double dt = util::durationToSec(currTime - state.timestamp);
		navtypes::Vectord<outputDim> lastTarget = state.lastTarget.value_or(currPosOutput);
		navtypes::Vectord<outputDim> currTarget = lastTarget + dt * state.targetVel;
		navtypes::Vectord<outputDim> outputPosDiff = currTarget - currPosOutput;
		navtypes::Matrixd<outputDim, inputDim> jacobian = jacobianFunc(currPos);
		navtypes::Vectord<inputDim> inputPosDiff =
			jacobian.colPivHouseholderQr().solve(outputPosDiff);

		// the cosine similarity of the actual and required position deltas in the output space
		// is metric of how much the kinematics allows the commanded movement.
		navtypes::Vectord<outputDim> trueOutputPosDiff = jacobian * inputPosDiff;
		cosineSim = outputPosDiff.dot(trueOutputPosDiff) /
					(outputPosDiff.norm() * trueOutputPosDiff.norm());

		state_t newState = {
			.timestamp = currTime, .targetVel = state.targetVel, .lastTarget = currTarget};
		controllerState = newState;

		return currPos + inputPosDiff;
	}

	/**
	 * @brief Set the target velocity.
	 *
	 * @param currTime The current timestamp.
	 * @param targetVel The target velocity, in the output space of the kinematic function.
	 */
	void setTarget(robot::types::datatime_t currTime,
				   const navtypes::Vectord<outputDim>& targetVel) {
		state_t state = {.timestamp = currTime, .targetVel = targetVel, .lastTarget = {}};
		this->controllerState = state;
	}

	/**
	 * @brief Check if a target velocity has been set.
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
		robot::types::datatime_t timestamp;
		navtypes::Vectord<outputDim> targetVel;
		std::optional<navtypes::Vectord<outputDim>> lastTarget;
	};

	std::function<navtypes::Vectord<outputDim>(const navtypes::Vectord<inputDim>&)>
		kinematicsFunc;
	std::function<navtypes::Matrixd<outputDim, inputDim>(const navtypes::Vectord<inputDim>&)>
		jacobianFunc;
	std::optional<state_t> controllerState;
};
