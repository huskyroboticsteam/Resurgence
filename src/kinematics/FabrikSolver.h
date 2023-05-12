#pragma once

#include "../navtypes.h"

#include <Eigen/Core>
#include <Eigen/Dense>

namespace kinematics {

/**
 * @brief Implementation of the FABRIK algorithm, in the special case of 2 dimensions.
 *
 * Note that joint constraints are only used to verify success at the end, they are not
 * used in intermediate parts of the algorithm.
 *
 * @tparam N The number of joints.
 *
 * @see http://www.andreasaristidou.com/FABRIK.html
 */
template <int N> class FabrikSolver2D {
public:
	/**
	 * @brief Construct an IK solver object.
	 *
	 * @param segLens The length of each arm segment.
	 * @param jointMin The minimum angle of each joint. Set to -pi for no limit.
	 * @param jointMax The maximum angle of each joint. Set to pi for no limit.
	 * @param thresh The IK solver will succeed when the EE position is off by at most this
	 * much.
	 * @param maxIter The maximum number of iterations to run the solver before failing.
	 */
	FabrikSolver2D(const navtypes::Arrayd<N>& segLens, const navtypes::Arrayd<N>& jointMin,
				   const navtypes::Arrayd<N>& jointMax, double thresh, int maxIter)
		: segLens(segLens), jointMin(jointMin), jointMax(jointMax), thresh(thresh),
		  maxIter(maxIter) {}

	/**
	 * @brief Solve for the joint angles that yield the given EE position.
	 *
	 * @param eePos The desired end effector position.
	 * @param jointAngles The current joint angles. Used as a starting guess for the IK solver.
	 * @param[out] success Output parameter that signals if the IK solver succeeded. Failure
	 * may occur if the target is unreachable or the IK solver exceeds the iteration limit.
	 * @return navtypes::Arrayd<N> The target joint angles that achieve the desired EE
	 * position.
	 */
	navtypes::Arrayd<N> solve(const Eigen::Vector2d& eePos,
							  const navtypes::Arrayd<N>& jointAngles, bool& success) const {
		double armLen = segLens.sum();
		if (eePos.norm() >= armLen) {
			success = false;
			return jointAngles;
		}

		navtypes::Arrayd<N + 1, 2> jointPos = jointAnglesToJointPos(jointAngles);
		success = false;
		for (int iter = 0; iter < maxIter; iter++) {
			// forward
			jointPos.row(N) = eePos;
			for (int i = N - 1; i >= 0; i--) {
				double jointDist = (jointPos.row(i) - jointPos.row(i + 1)).matrix().norm();
				double lambda = segLens[i] / jointDist;
				jointPos.row(i) =
					lambda * jointPos.row(i) + (1 - lambda) * jointPos.row(i + 1);
			}

			// backward
			jointPos.row(0) << 0, 0;
			for (int i = 0; i < N; i++) {
				double jointDist = (jointPos.row(i) - jointPos.row(i + 1)).matrix().norm();
				double lambda = segLens[i] / jointDist;
				jointPos.row(i + 1) =
					lambda * jointPos.row(i + 1) + (1 - lambda) * jointPos.row(i);
			}

			double err = (jointPos.row(N).matrix().transpose() - eePos).norm();
			if (err <= thresh) {
				success = true;
				break;
			}
		}

		navtypes::Arrayd<N> targetJointAngles = jointPosToJointAngles(jointPos);
		for (int i = 0; i < N; i++) {
			if (targetJointAngles[i] > jointMax[i] || targetJointAngles[i] < jointMin[i]) {
				success = false;
				return jointAngles;
			}
		}
		return targetJointAngles;
	}

private:
	navtypes::Arrayd<N> segLens;
	navtypes::Arrayd<N> jointMin;
	navtypes::Arrayd<N> jointMax;
	double thresh;
	int maxIter;

	navtypes::Arrayd<N + 1, 2>
	jointAnglesToJointPos(const navtypes::Arrayd<N>& jointAngles) const {
		std::array<double, N> jointAngleCumSum;
		jointAngleCumSum[0] = jointAngles[0];
		for (int i = 1; i < N; i++) {
			jointAngleCumSum[i] = jointAngleCumSum[i - 1] + jointAngles[i];
		}

		navtypes::Arrayd<N + 1, 2> jointPos;
		jointPos.row(0) << 0, 0;
		for (int i = 1; i <= N; i++) {
			Eigen::Vector2d seg;
			seg << std::cos(jointAngleCumSum[i - 1]), std::sin(jointAngleCumSum[i - 1]);
			jointPos.row(i) = jointPos.row(i - 1) + segLens[i - 1] * seg.array().transpose();
		}
		return jointPos;
	}

	navtypes::Arrayd<N>
	jointPosToJointAngles(const navtypes::Arrayd<N + 1, 2>& jointPos) const {
		navtypes::Arrayd<N + 1, 3> jointPos3d;
		jointPos3d << jointPos, navtypes::Arrayd<N + 1>::Zero();

		navtypes::Arrayd<N> jointAngles;
		jointAngles[0] = std::atan2(jointPos(1, 1), jointPos(1, 0));
		for (int i = 1; i < N; i++) {
			Eigen::RowVector3d prevSeg =
				(jointPos3d.row(i) - jointPos3d.row(i - 1)).matrix().normalized();
			Eigen::RowVector3d seg =
				(jointPos3d.row(i + 1) - jointPos3d.row(i)).matrix().normalized();

			jointAngles[i] = std::atan2(prevSeg.cross(seg).z(), prevSeg.dot(seg));
		}
		return jointAngles;
	}
};

} // namespace kinematics
