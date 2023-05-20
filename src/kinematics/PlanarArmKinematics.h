#pragma once

#include "../navtypes.h"
#include "FabrikSolver.h"

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
template <unsigned int N> class PlanarArmKinematics {
public:
	/**
	 * @brief Construct a new kinematics object.
	 *
	 * @param segLens The length of each arm segment.
	 * @param jointMin The minimum angle of each joint. Set to -pi for no limit.
	 * @param jointMax The maximum angle of each joint. Set to pi for no limit.
	 * @param solverThreshold The IK solver will succeed when the EE position is off by at most
	 * this much.
	 * @param solverMaxIter The maximum number of iterations to run the IK solver before
	 * failing.
	 */
	PlanarArmKinematics(const navtypes::Vectord<N>& segLens,
						const navtypes::Vectord<N>& jointMin,
						const navtypes::Vectord<N>& jointMax, double solverThreshold,
						int solverMaxIter)
		: segLens(segLens), jointMin(jointMin), jointMax(jointMax),
		  ikSolver(segLens, jointMin, jointMax, solverThreshold, solverMaxIter) {}

	/**
	 * @brief Get the number of segments in this arm.
	 *
	 * @return constexpr unsigned int The number of segments.
	 */
	constexpr unsigned int getNumSegments() const {
		return N;
	}

	/**
	 * @brief Get the segment lengths for this arm.
	 *
	 * @return The vector of segment lengths
	 */
	navtypes::Vectord<N> getSegLens() const {
		return segLens;
	}

	/**
	 * @brief Get the jacobian matrix for the arm at the given joint angles.
	 *
	 * @param jointPos The joint angles of the arm.
	 * @return navtypes::Matrixd<2, N> The jacobian matrix at the point.
	 * @see https://en.wikipedia.org/wiki/Jacobian_matrix_and_determinant
	 */
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

	/**
	 * @brief Given a joint position, calculate the current EE position.
	 *
	 * @param jointPos Current joint position.
	 * @return Eigen::Vector2d The EE position.
	 */
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

	/**
	 * @brief Calculate the EE velocity.
	 *
	 * @param jointPos Current joint position.
	 * @param jointVel Current velocity of each joint angle.
	 * @return Eigen::Vector2d The EE velocity.
	 */
	Eigen::Vector2d jointVelToEEVel(const navtypes::Vectord<N>& jointPos,
									const navtypes::Vectord<N>& jointVel) const {
		return getJacobian(jointPos) * jointVel;
	}

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
	navtypes::Vectord<N> eePosToJointPos(const Eigen::Vector2d& eePos,
										 const navtypes::Vectord<N>& currJointPos) const {
		bool success;
		return ikSolver.solve(eePos, currJointPos, success);
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
	navtypes::Vectord<N> eePosToJointPos(const Eigen::Vector2d& eePos,
										 const navtypes::Vectord<N>& currJointPos,
										 bool& success) const {
		return ikSolver.solve(eePos, currJointPos, success);
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

private:
	navtypes::Vectord<N> segLens;
	navtypes::Vectord<N> jointMin;
	navtypes::Vectord<N> jointMax;
	FabrikSolver2D<N> ikSolver;
};

} // namespace kinematics
