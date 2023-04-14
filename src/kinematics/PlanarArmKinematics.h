#pragma once

#include "../navtypes.h"

#include <array>
#include <initializer_list>

#include <Eigen/Core>

namespace kinematics {

template <unsigned int N> class PlanarArmKinematics {
public:
	explicit PlanarArmKinematics(const std::initializer_list<double>& segLens)
		: segLens(segLens) {}

	constexpr unsigned int getNumSegments() const {
		return N;
	}

	Eigen::Matrix<double, 2, N> getJacobian(const std::array<double, N>& jointPos) const {}

	Eigen::Vector2d jointPosToEEPos(const std::array<double, N>& jointPos) const {
		Eigen::Vector2d eePos = Eigen::Vector2d::Zero();
		double accumAngle = 0;
		for (int i = 0; i < N; i++) {
			accumAngle += jointPos[i];
			Eigen::Vector2d segVec{std::cos(accumAngle), std::sin(accumAngle)};
			eePos += segLens[i] * segVec;
		}
		return eePos;
	}

	Eigen::Vector2d jointVelToEEVel(const std::array<double, N>& jointPos,
									const std::array<double, N>& jointVel) const {
		navtypes::Vectord<N> jointVelVec;
		for (int i = 0; i < N; i++) {
			jointVelVec[i] = jointVel[i];
		}
		return jointVelToEEVel(jointPos, jointVelVec);
	}

	Eigen::Vector2d jointVelToEEVel(const std::array<double, N>& jointPos,
									const navtypes::Vectord<N>& jointVel) const {
		return getJacobian(jointPos) * jointVel;
	}

	std::array<double, N> eePosToJointPos(const Eigen::Vector2d& eePos) const {
		// TODO: implement
	}

	std::array<double, N> eeVelToJointVel(const Eigen::Vector2d& eePos,
										  const Eigen::Vector2d& eeVel) const {
		// TODO: implement
	}

private:
	std::array<double, N> segLens;
};

} // namespace kinematics
