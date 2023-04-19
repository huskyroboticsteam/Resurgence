#pragma once

#include "../navtypes.h"

#include <array>
#include <initializer_list>
#include <numeric>

#include <Eigen/Core>

namespace kinematics {

template <unsigned int N> class PlanarArmKinematics {
public:
	PlanarArmKinematics(const navtypes::Vectord<N>& segLens) : segLens(segLens) {}

	constexpr unsigned int getNumSegments() const {
		return N;
	}

	navtypes::Matrixd<2, N> getJacobian(const navtypes::Vectord<N>& jointPos) const {
		Eigen::Matrix<double, 2, N> jacobian = Eigen::Matrix<double, 2, N>::Zero();
		std::array<double, N> thetaCumSum;
		thetaCumSum[0] = jointPos[0];
		for (unsigned int i = 1; i < N; i++) {
			thetaCumSum[i] = jointPos[i] + thetaCumSum[i - 1];
		}
		for (unsigned int i = 0; i < N; i++) {
			for (unsigned int j = i; j < N; j++) {
				jacobian(0, i) += -segLens[j] * std::sin(thetaCumSum[j]);
				jacobian(1, i) += segLens[j] * std::cos(thetaCumSum[j]);
			}
		}
		return jacobian;
	}

	Eigen::Vector2d jointPosToEEPos(const navtypes::Vectord<N>& jointPos) const {
		Eigen::Vector2d eePos = Eigen::Vector2d::Zero();
		double accumAngle = 0;
		for (unsigned int i = 0; i < N; i++) {
			accumAngle += jointPos[i];
			Eigen::Vector2d segVec{std::cos(accumAngle), std::sin(accumAngle)};
			eePos += segLens[i] * segVec;
		}
		return eePos;
	}

	Eigen::Vector2d jointVelToEEVel(const navtypes::Vectord<N>& jointPos,
									const navtypes::Vectord<N>& jointVel) const {
		return getJacobian(jointPos) * jointVel;
	}

	navtypes::Vectord<N> eePosToJointPos(const Eigen::Vector2d& eePos) const {
		// TODO: implement
		return navtypes::Vectord<N>::Zero();
	}

	navtypes::Vectord<N> eeVelToJointVel(const Eigen::Vector2d& eePos,
										 const Eigen::Vector2d& eeVel) const {
		navtypes::Vectord<N> jointPos = eePosToJointPos(eePos);
		navtypes::Matrixd<2, N> jacobian = getJacobian(jointPos);
		return jacobian.colPivHouseholderQR().solve(eeVel);
	}

private:
	navtypes::Vectord<N> segLens;
};

} // namespace kinematics
