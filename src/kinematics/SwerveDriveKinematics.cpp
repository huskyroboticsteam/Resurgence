#include "SwerveDriveKinematics.h"

#include "../Util.h"

#include <Eigen/QR>
namespace kinematics { 
using namespace navtypes;
using util::toTransformRotateFirst;

SwerveDriveKinematics::SwerveDriveKinematics(double baseWidth, double baseLength)
	: baseWidth(baseWidth), baseLength(baseLength) {}

Eigen::Matrix<double, 8, 3> SwerveDriveKinematics::getIKMatrix() const {
	double lfx = baseLength / 2.0, rfx = lfx, lbx = -lfx, rbx = -lfx;
	double lfy = baseWidth / 2.0, rfy = -lfy, lby = lfy, rby = -lfy;

	Eigen::Matrix<double, 8, 3> M;

	M << 1, 0, -lfy, 
		0, 1, lfx, 
		1, 0, -rfy, 
		0, 1, rfx, 
		1, 0, -lby, 
		0, 1, lbx, 
		1, 0, -rby,
		0, 1, rbx;

	return M;
}

Eigen::Matrix<double, 8, 1>
SwerveDriveKinematics::getSwerveVelComponents(swervewheelvel_t wheelVel) const {
	Eigen::Matrix<double, 8, 1> res;

	res << wheelVel.lfVel * std::cos(wheelVel.lfRot), 
		wheelVel.lfVel * std::sin(wheelVel.lfRot),
		wheelVel.rfVel * std::cos(wheelVel.rfRot), 
		wheelVel.rfVel * std::sin(wheelVel.rfRot),
		wheelVel.lbVel * std::cos(wheelVel.lbRot), 
		wheelVel.lbVel * std::sin(wheelVel.lbRot),
		wheelVel.rbVel * std::cos(wheelVel.rbRot), 
		wheelVel.rbVel * std::sin(wheelVel.rbRot);

	return res;
}

pose_t SwerveDriveKinematics::wheelVelToRobotVel(swervewheelvel_t wheelVel) const {
	Eigen::Matrix<double, 8, 3> M = getIKMatrix();
	return M.colPivHouseholderQr().solve(getSwerveVelComponents(wheelVel));
}

swervewheelvel_t SwerveDriveKinematics::robotVelToWheelVel(double xVel, double yVel,
														double thetaVel) const {
	pose_t robotVels = {xVel, yVel, thetaVel};

	Eigen::Matrix<double, 8, 1> wheelVels = getIKMatrix() * robotVels;
	// M * robotVels returns column wheel velocity vectors {lfx, lfy, rfx, rfy, lbx, lby, rbx,
	// rby}

	return {std::hypot(wheelVels(1), wheelVels(0)), 
			std::atan2(wheelVels(1), wheelVels(0)),
			std::hypot(wheelVels(3), wheelVels(2)), 
			std::atan2(wheelVels(3), wheelVels(2)),
			std::hypot(wheelVels(5), wheelVels(4)), 
			std::atan2(wheelVels(5), wheelVels(4)),
			std::hypot(wheelVels(7), wheelVels(6)), 
			std::atan2(wheelVels(7), wheelVels(6))};
}

pose_t SwerveDriveKinematics::getLocalPoseUpdate(const swervewheelvel_t& wheelVel,
												double dt) const {
	pose_t robotVel = wheelVelToRobotVel(wheelVel);
	return robotVel * dt;
}

pose_t SwerveDriveKinematics::getPoseUpdate(const swervewheelvel_t& wheelVel, double heading,
											double dt) const {
	return toTransformRotateFirst(0, 0, -heading) * getLocalPoseUpdate(wheelVel, dt);
}

pose_t SwerveDriveKinematics::getNextPose(const swervewheelvel_t& wheelVel,
										const navtypes::pose_t& pose, double dt) const {
	return pose + getPoseUpdate(wheelVel, pose(2), dt);
}
}
