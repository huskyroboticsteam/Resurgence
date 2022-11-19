#include "SwerveDriveKinematics.h"

#include "../Util.h"

#include <cmath>

#include <Eigen/QR>

using namespace navtypes;
using util::toTransformRotateFirst;

SwerveDriveKinematics::SwerveDriveKinematics(double baseWidth, double baseLength)
	: baseWidth(baseWidth), baseLength(baseLength) {}

Eigen::MatrixXd SwerveDriveKinematics::getIKMatrix() const {
	double lfx = baseLength / 2.0, rfx = lfx, lbx = -lfx, rbx = -lfx;
	double lfy = baseWidth / 2.0, rfy = -lfy, lby = lfy, rby = -lfy;

	Eigen::MatrixXd M(8, 3);

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

Eigen::VectorXd
SwerveDriveKinematics::getSwerveVelComponents(swervewheelvel_t wheelVel) const {
	Eigen::VectorXd res(8);

	res << wheelVel.lfVel * cos(wheelVel.lfRot), 
           wheelVel.lfVel * sin(wheelVel.lfRot),
		   wheelVel.rfVel * cos(wheelVel.rfRot), 
           wheelVel.rfVel * sin(wheelVel.rfRot),
		   wheelVel.lbVel * cos(wheelVel.lbRot), 
           wheelVel.lbVel * sin(wheelVel.lbRot),
		   wheelVel.rbVel * cos(wheelVel.rbRot), 
           wheelVel.rbVel * sin(wheelVel.rbRot);

	return res;
}

pose_t SwerveDriveKinematics::wheelVelToRobotVel(swervewheelvel_t wheelVel) const {
	Eigen::MatrixXd M(8, 3);
	M = getIKMatrix();
	return M.colPivHouseholderQr().solve(getSwerveVelComponents(wheelVel));
}

swervewheelvel_t SwerveDriveKinematics::robotVelToWheelVel(double xVel, double yVel,
														   double thetaVel) const {
	pose_t robotVels = {xVel, yVel, thetaVel};

	Eigen::VectorXd wheelVels(8);
	wheelVels = getIKMatrix() * robotVels;
	// M * robotVels returns column wheel velocity vectors {lfx, lfy, rfx, rfy, lbx, lby, rbx,
	// rby}

	return {hypot(wheelVels(1), wheelVels(0)), 
            atan2(wheelVels(1), wheelVels(0)),
			hypot(wheelVels(3), wheelVels(2)), 
            atan2(wheelVels(3), wheelVels(2)),
			hypot(wheelVels(5), wheelVels(4)), 
            atan2(wheelVels(5), wheelVels(4)),
			hypot(wheelVels(7), wheelVels(6)), 
            atan2(wheelVels(7), wheelVels(6))};
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
