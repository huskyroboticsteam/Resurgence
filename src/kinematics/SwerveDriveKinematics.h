#pragma once

#include "../navtypes.h"

using namespace navtypes;

struct swervewheelvel_t {
	double lfVel;
	double lfRot;
	double rfVel;
	double rfRot;
	double lbVel;
	double lbRot;
	double rbVel;
	double rbRot;
};

class SwerveDriveKinematics {
public:
	explicit SwerveDriveKinematics(double baseWidth, double baseLength);
	swervewheelvel_t robotVelToWheelVel(double xVel, double yVel, double thetaVel) const;
	pose_t wheelVelToRobotVel(swervewheelvel_t wheelVel) const;
	navtypes::pose_t getLocalPoseUpdate(const swervewheelvel_t& wheelVel, double dt) const;
	navtypes::pose_t getPoseUpdate(const swervewheelvel_t& wheelVel, double heading,
								   double dt) const;
	navtypes::pose_t getNextPose(const swervewheelvel_t& wheelVel,
								 const navtypes::pose_t& pose, double dt) const;

private:
	double baseWidth;
	double baseLength;
	Eigen::MatrixXd getIKMatrix() const;
	Eigen::VectorXd getSwerveVelComponents(swervewheelvel_t wheelVel) const;
};