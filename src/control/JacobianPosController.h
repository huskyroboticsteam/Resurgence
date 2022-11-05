#pragma once

#include "../filters/StateSpaceUtil.h"
#include "../navtypes.h"
#include "../world_interface/data.h"
#include "TrapezoidalVelocityProfile.h"

#include <array>
#include <functional>
#include <optional>

#include <Eigen/Core>

namespace control {

/**
 * @brief This class controls the position of a multidimensional mechanism
 * with a trapezoidal velocity profile using the jacobian of the kinematics.
 *
 * @tparam outputDim The dimension of the output vector of the kinematic function.
 * @tparam inputDim The dimension of the input vector of the kinematic function.
 * @see https://en.wikipedia.org/wiki/Jacobian_matrix_and_determinant
 */
template <int outputDim, int inputDim> class JacobianPosController {
public:
	/**
	 * @brief Construct a new controller.
	 *
	 * @param kinematicsFunc The kinematics of the mechanism being controlled. This is
	 * a function that takes in some input vector (ex: joint angles of an arm) and outputs
	 * another vector (ex: the 3d pose of the hand)
	 * @param jacobianFunc The jacobian of the kinematic function.
	 * When executing a command, getCommand() should be called at this rate.
	 * @param maxVels The maximum velocity in the output space of the kinematic function.
	 * @param maxAccels The maximum acceleration in the output space of the kinematic function.
	 */
	JacobianPosController(
		const std::function<navtypes::Vectord<outputDim>(const navtypes::Vectord<inputDim>&)>&
			kinematicsFunc,
		const std::function<navtypes::Matrixd<outputDim, inputDim>(
			const navtypes::Vectord<inputDim>&)>& jacobianFunc,
		double maxVel, double maxAccel)
		: kinematicsFunc(kinematicsFunc),
		  jacobianFunc(jacobianFunc
						   ? jacobianFunc
						   : std::bind(filters::statespace::numericalJacobian, kinematicsFunc,
									   std::placeholders::_1, outputDim)),
		  velocityProfile(maxVel, maxAccel) {
		assert(this->kinematicsFunc);
		assert(this->jacobianFunc);
	}

	/**
	 * @brief Get the current command position in the input space.
	 *
	 * @param currTime The current timestamp.
	 * @param currPos The current position, in the input space.
	 * @return navtypes::Vectord<inputDim> The target position, in the input space of the
	 * kinematic function. Returns the current position if no target is set.
	 */
	navtypes::Vectord<inputDim> getCommand(robot::types::datatime_t currTime,
										   const navtypes::Vectord<inputDim>& currPos) const {
		double unused;
		return getCommand(currTime, currPos, unused);
	}

	/**
	 * @brief Get the current command position in the input space.
	 *
	 * @param currTime The current timestamp.
	 * @param currPos The current position, in the input space.
	 * @param[out] cosineSim Output parameter, gives the cosine similarity of the delta to the
	 * returned target and the delta to the commanded target. This similarity may be low if the
	 * kinematics of the mechanism do not allow it to move in the commanded direction. May be
	 * NaN if stuck in singularity point, in which case the command is the current position.
	 * @return navtypes::Vectord<inputDim> The target position, in the input space of the
	 * kinematic function. Returns the current position if no target is set.
	 */
	navtypes::Vectord<inputDim> getCommand(robot::types::datatime_t currTime,
										   const navtypes::Vectord<inputDim>& currPos,
										   double& cosineSim) const {
		if (!velocityProfile.hasTarget()) {
			return currPos;
		}

		// The jacobian of the kinematics function represents a linearization of the kinematics
		// around the current position. We then use this approximation to find the position
		// delta in the input space that yields the required position delta in the output space

		navtypes::Vectord<outputDim> currTarget = velocityProfile.getCommand(currTime);
		navtypes::Vectord<outputDim> currPosOutput = kinematicsFunc(currPos);
		navtypes::Vectord<outputDim> outputPosDiff = currTarget - currPosOutput;
		navtypes::Matrixd<outputDim, inputDim> jacobian = jacobianFunc(currPos);
		navtypes::Vectord<inputDim> inputPosDiff =
			jacobian.colPivHouseholderQr().solve(outputPosDiff);

		// the cosine similarity of the actual and required position deltas in the output space
		// is metric of how much the kinematics allows the commanded movement.
		navtypes::Vectord<outputDim> trueOutputPosDiff = jacobian * inputPosDiff;
		cosineSim = outputPosDiff.dot(trueOutputPosDiff) /
					(outputPosDiff.norm() * trueOutputPosDiff.norm());

		return currPos + inputPosDiff;
	}

	/**
	 * @brief Set the target position of the controller.
	 *
	 * @param currTime The current timestamp.
	 * @param currPos The current position in the input space.
	 * @param target The target position in the output space.
	 */
	void setTarget(robot::types::datatime_t currTime,
				   const navtypes::Vectord<inputDim>& currPos,
				   const navtypes::Vectord<outputDim>& target) {
		navtypes::Vectord<outputDim> currPosOutput = kinematicsFunc(currPos);
		velocityProfile.setTarget(currTime, currPosOutput, target);
	}

	/**
	 * @brief Check if a target has been set.
	 *
	 * @return bool True iff a target has been set.
	 */
	bool hasTarget() const {
		return velocityProfile.hasTarget();
	}

	/**
	 * @brief Reset the controller, unsetting the target.
	 */
	void reset() {
		velocityProfile.reset();
	}

private:
	std::function<navtypes::Vectord<outputDim>(const navtypes::Vectord<inputDim>&)>
		kinematicsFunc;
	std::function<Eigen::Matrix<double, outputDim, inputDim>(
		const Eigen::Matrix<double, inputDim, 1>&)>
		jacobianFunc;
	TrapezoidalVelocityProfile<outputDim> velocityProfile;
};

} // namespace control
