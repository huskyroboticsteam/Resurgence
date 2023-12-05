#pragma once

#include "../navtypes.h"
namespace kinematics {
struct wheelvel_t {
	double lVel;
	double rVel;
};

/**
 * Represents the kinematics of a differential drive. This class can be used to calculate
 * various velocities and pose updates using the mathematical representation of the drivebase.
 */
class DiffDriveKinematics {
public:
	/**
	 * Choose whether the velocity should be scaled proportionally, with respect to x velocity,
	 * or with respect to theta velocity.
	 */
	enum class PreferredVelPreservation {
		Proportional,
		PreferXVel,
		PreferThetaVel
	};

	/**
	 * Create a new DiffDriveKinematics with the given wheel base width.
	 *
	 * @param wheelBaseWidth The width of the wheelbase. The units themselves don't matter, as
	 * long as you're consistent.
	 */
	explicit DiffDriveKinematics(double wheelBaseWidth);

	/**
	 * Given the left and right wheel velocities, find the robot velocity in the robot's
	 * reference frame. Since this is a non-holonomic drivebase, the y velocity will be 0.
	 *
	 * @param lVel The tangential velocity of the left wheel.
	 * @param rVel The tangential velocity of the right wheel.
	 * @return The robot velocity in the form [xVel, yVel, thetaVel] in the robot's local
	 * reference frame. thetaVel will be in units rad/sec.
	 */
	navtypes::pose_t wheelVelToRobotVel(double lVel, double rVel) const;

	/**
	 * Given the robot velocity in the robot's local reference frame, find the left and right
	 * tangential wheel velocities.
	 *
	 * @param xVel The x velocity in the robot's reference frame.
	 * @param thetaVel The rotational velocity around the z-axis. In rad/sec.
	 * @return Struct with fields lVel and rVel, representing the left and right velocities.
	 */
	wheelvel_t robotVelToWheelVel(double xVel, double thetaVel) const;

	/**
	 * Calculate the pose update in the local reference frame assuming the left and right
	 * wheels have constant velocity.
	 *
	 * @param wheelVel The velocities of the wheels.
	 * @param dt The elapsed time in seconds for which the update is calculated. The robot
	 * has been moving at the specified velocity for this much time.
	 * @return The pose update in the robot's local reference frame.
	 */
	navtypes::pose_t getLocalPoseUpdate(const wheelvel_t& wheelVel, double dt) const;

	/**
	 * Calculate the pose update in the global reference frame (map space) assuming the left
	 * and right wheels have constant velocity.
	 *
	 * @param wheelVel The velocities of the wheels.
	 * @param dt The elapsed time in seconds for which the update is calculated. The robot
	 * has been moving at the specified velocity for this much time.
	 * @return The pose update in the global reference frame.
	 */
	navtypes::pose_t getPoseUpdate(const wheelvel_t& wheelVel, double heading,
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
	navtypes::pose_t getNextPose(const wheelvel_t& wheelVel, const navtypes::pose_t& pose,
								 double dt) const;

	/**
	 * Ensure that the given xVel and thetaVel translate to a wheel speed less than or equal
	 * to the max wheel speed. Scales them if they do not.
	 *
	 * @param preferred Choose proportional if x velocity and theta velocity should be
	 *                  scaled proportionally to each other, choose PreferXVel if linear
	 * velocity is preferred, or choose PreferThetaVel if turning is preferred.
	 * @param xVel The xVel used to calculate the wheel speed
	 * @param thetaVel The theta veloicty used to calculate the wheel speed.
	 * @param maxWheelSpeed The current max possible wheel speed. This is the value
	 *                      that the calculated velocity will be checked against.
	 */
	navtypes::pose_t ensureWithinWheelSpeedLimit(PreferredVelPreservation preferred,
												 double xVel, double thetaVel,
												 double maxWheelSpeed) const;

private:
	double wheelBaseWidth;
};
} // namespace kinematics
