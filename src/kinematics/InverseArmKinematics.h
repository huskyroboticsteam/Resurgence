#pragma once

#include "../navtypes.h"

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
	 * @return navtypes::Arrayd<N> The target joint angles that achieve the desired EE
	 * position.
	 */
	virtual navtypes::Vectord<N> eePosToJointPos(const navtypes::Vectord<D>& eePos,
												 const navtypes::Vectord<N>& jointAngles,
												 bool& success) const = 0;
};

} // namespace kinematics
