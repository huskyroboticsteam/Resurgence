#pragma once

#include "../navtypes.h"

#include <Eigen/Core>

namespace kinematics {

/**
 * @brief A class that calculates the forward kinematics of an arm.
 *
 * @tparam D The number of dimensions of the end effector space.
 * @tparam N The number of joints in the arm.
 */
template <unsigned int D, unsigned int N>
class ForwardArmKinematics {
public:
	virtual ~ForwardArmKinematics() = default;

	/**
	 * @brief Get the number of segments in this arm.
	 *
	 * @return constexpr unsigned int The number of segments.
	 */
	constexpr unsigned int getNumSegments() const {
		return N;
	}

	/**
	 * @brief Get the number of dimensions of the end effector space.
	 *
	 * @return constexpr unsigned int The number of dimensions.
	 */
	constexpr unsigned int getNumDimensions() const {
		return D;
	}

	/**
	 * @brief Get the segment lengths for this arm.
	 *
	 * @return The vector of segment lengths
	 */
	virtual navtypes::Vectord<N> getSegLens() const = 0;

	/**
	 * @brief Check if the given joint configuration is valid.
	 *
	 * @param jointPos The joint positions to check.
	 * @return true iff the joint configuration is valid, false otherwise.
	 */
	virtual bool satisfiesConstraints(const navtypes::Vectord<N>& jointPos) const = 0;

	/**
	 * @brief Get the jacobian matrix for the arm at the given joint angles.
	 *
	 * @param jointPos The joint angles of the arm.
	 * @return navtypes::Matrixd<D, N> The jacobian matrix at the point.
	 * @see https://en.wikipedia.org/wiki/Jacobian_matrix_and_determinant
	 */
	virtual navtypes::Matrixd<D, N>
	getJacobian(const navtypes::Vectord<N>& jointPos) const = 0;

	/**
	 * @brief Given a joint position, calculate the current EE position.
	 *
	 * @param jointPos Current joint position.
	 * @return Eigen::Vectord<D> The EE position.
	 */
	virtual navtypes::Vectord<D>
	jointPosToEEPos(const navtypes::Vectord<N>& jointPos) const = 0;

	/**
	 * @brief Calculate the EE velocity.
	 *
	 * @param jointPos Current joint position.
	 * @param jointVel Current velocity of each joint angle.
	 * @return Eigen::Vectord<D> The EE velocity.
	 */
	navtypes::Vectord<D> jointVelToEEVel(const navtypes::Vectord<N>& jointPos,
										 const navtypes::Vectord<N>& jointVel) const {
		return getJacobian(jointPos) * jointVel;
	}

	/**
	 * @brief Calculates the joint velocity that yields the desired EE velocity.
	 *
	 * @param jointPos The current joint angles.
	 * @param eeVel The desired EE velocity.
	 * @return navtypes::Vectord<N> The joint velocities that yields the desired EE velocity.
	 */
	navtypes::Vectord<N> eeVelToJointVel(const navtypes::Vectord<N>& jointPos,
										 const Eigen::Vector2d& eeVel) const {
		navtypes::Matrixd<2, N> jacobian = getJacobian(jointPos);
		return jacobian.colPivHouseholderQR().solve(eeVel);
	}
};

} // namespace kinematics
