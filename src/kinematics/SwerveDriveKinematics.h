#pragma once

#include "../navtypes.h"

using namespace navtypes;

struct swervewheelvel_t {
	/**
	 * Represents robot wheel velocities in robot reference frame,
	 * in polar form (wheel speed and angle)
	 */
	
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
	/**
	 * Create a new SwerveDriveKinematics with the given wheel base width and length from rear of robot.
	 *
	 * @param baseWidth The width of the wheelbase. The units themselves don't matter, as
	 * long as you're consistent.
	 * @param baseLength The length of the wheelbase. The units themselves don't matter, as
	 * long as you're consistent.
	 */
	explicit SwerveDriveKinematics(double baseWidth, double baseLength);

	/**
	 * Given the robot's translational and angular velocity in the robot's
	 * reference frame, find the speeds and angles of the robot's wheels in the robot's
	 * reference frame.
	 *
	 * @param xVel the forward velocity component of the robot, in the robot's reference frame. (positive = front)
	 * @param yVel the left/right velocity component of the robot, in the robot's reference frame. (positive = left)
	 * @param thetaVel the angular velocity of the robot in the robot's reference frame, in rad/sec. (positive = CCW)
	 * @return the speeds and angles of the robot's wheels in the robot's reference frame.
	 */
	swervewheelvel_t robotVelToWheelVel(double xVel, double yVel, double thetaVel) const;

	/**
	 * Given the speeds and angles of the robot's wheels in the robot's reference frame,
	 * find the robot's translational velocity and angular velocity in the robot's reference frame.
	 * 
	 * @param wheelVel the struct containing the speeds and angles of the robot's wheels in the robot's reference frame.
	 * @return the robot's translational and angular velocity in the robot's reference frame.
	 */
	pose_t wheelVelToRobotVel(swervewheelvel_t wheelVel) const;

	/**
	 * Calculate the pose update in the local reference frame given the velocities of the robot's
	 * wheels and the time step.
	 *
	 * @param wheelVel The velocities of the wheels.
	 * @param dt The elapsed time in seconds for which the update is calculated. The robot
	 * has been moving at the specified velocity for this much time.
	 * @return The pose update in the robot's local reference frame.
	 */
	navtypes::pose_t getLocalPoseUpdate(const swervewheelvel_t& wheelVel, double dt) const;


	/**
	 * Calculate the pose update in the global reference frame given the velocities of the robot's
	 * wheels and the time step.
	 *
	 * @param wheelVel The velocities of the wheels.
	 * @param heading The heading of the robot, in radians.
	 * @param dt The elapsed time in seconds for which the update is calculated. The robot
	 * has been moving at the specified velocity for this much time.
	 * @return The pose update in the global reference frame.
	 */
	navtypes::pose_t getPoseUpdate(const swervewheelvel_t& wheelVel, double heading,
								   double dt) const;

	/**
	 * Calculate the next pose of the robot in the global reference frame. (map space)
	 *
	 * @param wheelVel The velocities of the wheels.
	 * @param pose The current pose of the robot, in the global reference frame.
	 * @param dt The elapsed time in seconds for which the update is calculated. The robot
	 * has been moving at the specified velocity for this much time.
	 * @return The next pose of the robot in the global reference frame after having moved at
	 * the specified velocity for the specified time.
	 */
	navtypes::pose_t getNextPose(const swervewheelvel_t& wheelVel,
								 const navtypes::pose_t& pose, double dt) const;

private:
	double baseWidth;
	double baseLength;

	/**
	 * Computes the Inverse Kinematics matrix of the robot that, when multipled by a 3x1
	 * vector of the robot's translational and angular velocities, gives the 8x1 vector of
	 * wheel velocity components of the robot.
	 */
	Eigen::MatrixXd getIKMatrix() const;

	/**
	 * Converts the polar speed data of wheelVel (wheel speed of each wheel and direction of each wheel)
	 * into rectangular velocity components of the robot's wheels, both in the robot's reference frame.
	 * 
	 * @param wheelVel the robot's wheel velocities in the robot's reference frame in polar format.
	 * @return a 8x1 vector of the robot's rectangular wheel velocity components in the robot's reference frame.
	 */
	Eigen::VectorXd getSwerveVelComponents(swervewheelvel_t wheelVel) const;
};