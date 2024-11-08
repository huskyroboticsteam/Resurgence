#pragma once

#include "../navtypes.h"
#include "InverseArmKinematics.h"
#include "PlanarArmFK.h"

#include <memory>

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
template <int N>
class FabrikSolver2D : public InverseArmKinematics<2, N> {
public:
	/**
	 * @brief Construct an IK solver object.
	 *
	 * @param fk The forward kinematics of the arm.
	 * @param thresh The IK solver will succeed when the EE position is off by at most this
	 * much.
	 * @param maxIter The maximum number of iterations to run the solver before failing.
	 */
	FabrikSolver2D(std::shared_ptr<const PlanarArmFK<N>> fk, double thresh, int maxIter)
		: InverseArmKinematics<2, N>(fk), segLens(fk->getSegLens()), thresh(thresh),
		  maxIter(maxIter) {}

	navtypes::Vectord<N> solve(const Eigen::Vector2d& eePos,
							   const navtypes::Vectord<N>& jointAngles,
							   bool& success) const override {
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

		navtypes::Vectord<N> targetJointAngles = jointPosToJointAngles(jointPos);
		return targetJointAngles;
	}

private:
	navtypes::Arrayd<N> segLens;
	double thresh;
	int maxIter;

	navtypes::Arrayd<N + 1, 2>
	jointAnglesToJointPos(const navtypes::Vectord<N>& jointAngles) const {
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

	navtypes::Vectord<N>
	jointPosToJointAngles(const navtypes::Arrayd<N + 1, 2>& jointPos) const {
		navtypes::Arrayd<N + 1, 3> jointPos3d;
		jointPos3d << jointPos, navtypes::Arrayd<N + 1>::Zero();

		navtypes::Vectord<N> jointAngles;
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
