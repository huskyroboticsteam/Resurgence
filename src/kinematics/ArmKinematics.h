#pragma once

#include "ForwardArmKinematics.h"
#include "InverseArmKinematics.h"

#include <memory>

#include <Eigen/Core>

namespace kinematics {

/**
 * @brief A class that combines the forward and inverse kinematics of an arm.
 *
 * @tparam D The number of dimensions of the end effector space.
 * @tparam N The number of joints in the arm.
 */
template <unsigned int D, unsigned int N>
class ArmKinematics : public ForwardArmKinematics<D, N>, public InverseArmKinematics<D, N> {
public:
	ArmKinematics(std::shared_ptr<const ForwardArmKinematics<D, N>> forwardKin,
				  std::shared_ptr<const InverseArmKinematics<D, N>> inverseKin)
		: fk(forwardKin), ik(inverseKin) {}

	navtypes::Vectord<N> getSegLens() const override {
		return fk->getSegLens();
	}

	navtypes::Vectord<D> jointPosToEEPos(const navtypes::Vectord<N>& jointPos) const override {
		return fk->jointPosToEEPos(jointPos);
	}

	navtypes::Vectord<N> eePosToJointPos(const navtypes::Vectord<D>& eePos,
										 const navtypes::Vectord<N>& jointAngles,
										 bool& success) const override {
		return ik->eePosToJointPos(eePos, jointAngles, success);
	}

	navtypes::Matrixd<D, N> getJacobian(const navtypes::Vectord<N>& jointPos) const override {
		return fk->getJacobian(jointPos);
	}

private:
	const std::shared_ptr<const ForwardArmKinematics<D, N>> fk;
	const std::shared_ptr<const InverseArmKinematics<D, N>> ik;
};

} // namespace kinematics
