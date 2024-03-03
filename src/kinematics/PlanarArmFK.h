#pragma once

#include "../navtypes.h"
#include "FabrikSolver.h"
#include "ForwardArmKinematics.h"

#include <array>
#include <initializer_list>
#include <numeric>

#include <Eigen/Core>

namespace kinematics {

/**
 * @brief Kinematics object for a sequence of arm segments in a 2d plane.
 *
 * @tparam N The number of joints.
 */
template <unsigned int N>
class PlanarArmFK : public ForwardArmKinematics<2, N> {
public:
	/**
	 * @brief Construct a new kinematics object.
	 *
	 * @param segLens The length of each arm segment.
	 * @param jointMin The minimum angle of each joint. Set to -pi for no limit.
	 * @param jointMax The maximum angle of each joint. Set to pi for no limit.
	 */
	PlanarArmFK(const navtypes::Vectord<N>& segLens,
						const navtypes::Vectord<N>& jointMin,
						const navtypes::Vectord<N>& jointMax)
		: segLens(segLens), jointMin(jointMin), jointMax(jointMax) {}

	navtypes::Vectord<N> getSegLens() const override {
		return segLens;
	}

	navtypes::Matrixd<2, N> getJacobian(const navtypes::Vectord<N>& jointPos) const override {
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

	Eigen::Vector2d jointPosToEEPos(const navtypes::Vectord<N>& jointPos) const override {
		Eigen::Vector2d eePos = Eigen::Vector2d::Zero();
		double accumAngle = 0;
		for (unsigned int i = 0; i < N; i++) {
			accumAngle += jointPos[i];
			Eigen::Vector2d segVec{std::cos(accumAngle), std::sin(accumAngle)};
			eePos += segLens[i] * segVec;
		}
		return eePos;
	}

private:
	navtypes::Vectord<N> segLens;
	navtypes::Vectord<N> jointMin;
	navtypes::Vectord<N> jointMax;
};

} // namespace kinematics
