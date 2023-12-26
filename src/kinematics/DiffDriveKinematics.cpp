#include "DiffDriveKinematics.h"

#include "../utils/transform.h"

using namespace navtypes;
using util::toTransformRotateFirst;

namespace kinematics {
DiffDriveKinematics::DiffDriveKinematics(double wheelBaseWidth)
	: wheelBaseWidth(wheelBaseWidth) {}

pose_t DiffDriveKinematics::wheelVelToRobotVel(double lVel, double rVel) const {
	return {(lVel + rVel) / 2.0, 0, (rVel - lVel) / wheelBaseWidth};
}

wheelvel_t DiffDriveKinematics::robotVelToWheelVel(double xVel, double thetaVel) const {
	return {xVel - 0.5 * wheelBaseWidth * thetaVel, xVel + 0.5 * wheelBaseWidth * thetaVel};
}

pose_t DiffDriveKinematics::getLocalPoseUpdate(const wheelvel_t& wheelVel, double dt) const {
	double lVel = wheelVel.lVel;
	double rVel = wheelVel.rVel;

	// Instead of using euler integration, we can represent the pose update as a twist
	// and then apply that twist to the current pose. Derived from differential drive
	// kinematics. This will be more accurate than euler integration at slower update rates.

	double forward = (lVel + rVel) * 0.5 * dt; // distance the robot drove along a circle
	double dTheta = (rVel - lVel) / wheelBaseWidth * dt; // the change in heading

	double dx, dy;
	// if dTheta is close to 0 use the taylor approximations
	if (fabs(dTheta) <= 1e-5) {
		dx = forward * (1.0 - dTheta * dTheta / 6.0);
		dy = forward * 0.5 * dTheta;
	} else {
		dx = forward * sin(dTheta) / dTheta;
		dy = forward * (1 - cos(dTheta)) / dTheta;
	}

	return {dx, dy, dTheta};
}

pose_t DiffDriveKinematics::getPoseUpdate(const wheelvel_t& wheelVel, double heading,
										  double dt) const {
	return toTransformRotateFirst(0, 0, -heading) * getLocalPoseUpdate(wheelVel, dt);
}

pose_t DiffDriveKinematics::getNextPose(const wheelvel_t& wheelVel, const pose_t& pose,
										double dt) const {
	return pose + getPoseUpdate(wheelVel, pose(2), dt);
}

pose_t DiffDriveKinematics::ensureWithinWheelSpeedLimit(
	PreferredVelPreservation preferredVelPreservation, double xVel, double thetaVel,
	double maxWheelSpeed) const {
	auto [lVel, rVel] = robotVelToWheelVel(xVel, thetaVel);
	double calculatedMaxWheelVel = std::max(std::abs(lVel), std::abs(rVel));
	if (calculatedMaxWheelVel > maxWheelSpeed) {
		switch (preferredVelPreservation) {
			case PreferredVelPreservation::Proportional: {
				xVel *= maxWheelSpeed / calculatedMaxWheelVel;
				thetaVel *= maxWheelSpeed / calculatedMaxWheelVel;
			}
			case PreferredVelPreservation::PreferXVel: {
				if (std::abs(xVel) > maxWheelSpeed) {
					return {std::copysign(maxWheelSpeed, xVel), 0, 0};
				}
				// Bring lVel up to -maxWheelSpeed
				if (lVel < -maxWheelSpeed) {
					thetaVel = (-maxWheelSpeed - xVel) * -2.0 / wheelBaseWidth;
				}
				// Bring lVel down to maxWheelSpeed
				if (lVel > maxWheelSpeed) {
					thetaVel = (maxWheelSpeed - xVel) * -2.0 / wheelBaseWidth;
				}
				// Bring rVel up to -maxWheelSpeed
				if (rVel < -maxWheelSpeed) {
					thetaVel = (-maxWheelSpeed - xVel) * 2.0 / wheelBaseWidth;
				}
				// Bring rVel down to maxWheelSpeed
				if (rVel > maxWheelSpeed) {
					thetaVel = (maxWheelSpeed - xVel) * 2.0 / wheelBaseWidth;
				}
				break;
			}
			case PreferredVelPreservation::PreferThetaVel: {
				if (std::abs(wheelBaseWidth / 2 * thetaVel) > maxWheelSpeed) {
					return {0, 0, std::copysign(maxWheelSpeed * 2 / wheelBaseWidth, thetaVel)};
				}
				if (std::max(lVel, rVel) > maxWheelSpeed) {
					xVel -= std::max(lVel, rVel) - maxWheelSpeed;
				}
				if (std::min(lVel, rVel) < -maxWheelSpeed) {
					xVel += -maxWheelSpeed - std::min(lVel, rVel);
				}
				break;
			}
		}
	}
	return {xVel, 0, thetaVel};
}

} // namespace kinematics
