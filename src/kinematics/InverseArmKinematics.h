#pragma once

#include "../navtypes.h"
#include "ForwardArmKinematics.h"

#include <memory>

#include <Eigen/Core>

namespace kinematics {

/**
 * @brief A class that calculates the inverse kinematics of an arm.
 *
 * @tparam D The number of dimensions of the end effector space.
 * @tparam N The number of joints in the arm.
 */
template <unsigned int D, unsigned int N>
class InverseArmKinematics {
public:
	/**
	 * @brief Construct an IK object.
	 *
	 * @param fk The forward kinematics that are inverted by this class.
	 */
	InverseArmKinematics(std::shared_ptr<const ForwardArmKinematics<D, N>> fk) : fk(fk) {}

	virtual ~InverseArmKinematics() = default;

	/**
	 * @brief Calculate the joint angles that yield the desired EE position.
	 *
	 * @warning This may fail, for multiple reasons. To check for success, use the overload of
	 * this method with the @p success parameter.
	 *
	 * @param eePos Desired EE position.
	 * @param currJointPos The current joint angles.
	 * @return navtypes::Vectord<N> Joint angles that yield the desired EE position.
	 */
	navtypes::Vectord<N> eePosToJointPos(const navtypes::Vectord<D>& eePos,
										 const navtypes::Vectord<N>& currJointPos) const {
		bool success;
		return eePosToJointPos(eePos, currJointPos, success);
	}

	/**
	 * @brief Solve for the joint angles that yield the given EE position.
	 *
	 * @param eePos The desired end effector position.
	 * @param currJointPos The current joint angles. Used as a starting guess for the IK
	 * solver.
	 * @param[out] success Output parameter that signals if the IK solver succeeded. Failure
	 * may occur if the target is unreachable or the IK solver exceeds the iteration limit.
	 * @return navtypes::Vectord<N> The target joint angles that achieve the desired EE
	 * position.
	 */
	virtual navtypes::Vectord<N> eePosToJointPos(const navtypes::Vectord<D>& eePos,
												 const navtypes::Vectord<N>& jointAngles,
												 bool& success) const {
		double armLen = fk->getSegLens().sum();
		if (eePos.norm() >= armLen) {
			success = false;
			return jointAngles;
		}
		navtypes::Vectord<N> sol = solve(eePos, jointAngles, success);
		if (success) {
			success = fk->satisfiesConstraints(sol);
		}
		return success ? sol : jointAngles;
	}

protected:
	const std::shared_ptr<const ForwardArmKinematics<D, N>> fk;

	/**
	 * @brief Solve for the joint angles that yield the given EE position.
	 *
	 * This method does not need to verify that the solution verifies the FK constraints,
	 * or if the target position is within the arm's reach.
	 *
	 * @param eePos The desired end effector position.
	 * @param currJointPos The current joint angles. Used as a starting guess for the IK
	 * solver.
	 * @param[out] success Output parameter that signals if the IK solver succeeded. Failure
	 * may occur if the target is unreachable or the IK solver exceeds the iteration limit.
	 * @return navtypes::Vectord<N> The target joint angles that achieve the desired EE
	 * position.
	 */
	virtual navtypes::Vectord<N> solve(const navtypes::Vectord<D>& eePos,
									   const navtypes::Vectord<N>& jointAngles,
									   bool& success) const = 0;
};

} // namespace kinematics
